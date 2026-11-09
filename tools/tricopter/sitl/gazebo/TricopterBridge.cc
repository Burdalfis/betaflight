// Articulated three-motor Betaflight SITL plant / UDP bridge.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "TricopterMath.hh"
#include "MotorModel.hh"
#include "DragModel.hh"

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Static.hh>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace tricopter
{
using Vec = gz::math::Vector3d;
using Clock = std::chrono::steady_clock;

class TricopterBridge final : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate,
    public gz::sim::ISystemReset
{
public:
  ~TricopterBridge() override
  {
    if (socketFd >= 0) close(socketFd);
  }

  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    try
    {
      const gz::sim::Model model(entity);
      cameraHeadingName = sdf->Get<std::string>("camera_heading_model", "").first;
      if (const auto *parent = ecm.Component<gz::sim::components::ParentEntity>(entity))
        worldEntity = parent->Data();
      body = gz::sim::Link(model.LinkByName(ecm, sdf->Get<std::string>(
          "body_link", "base_link").first));
      joint = gz::sim::Joint(model.JointByName(ecm, sdf->Get<std::string>(
          "servo_joint", "tail_joint").first));
      const std::array<std::string, 3> defaults {"tail_motor", "right_motor", "left_motor"};
      for (unsigned i = 0; i < 3; ++i)
      {
        rotor[i] = gz::sim::Link(model.LinkByName(ecm, sdf->Get<std::string>(
            "motor" + std::to_string(i) + "_link", defaults[i]).first));
        if (rotor[i].Entity() == gz::sim::kNullEntity)
          throw std::runtime_error("Rotor link not found");
        rotor[i].EnableVelocityChecks(ecm);
      }
      if (body.Entity() == gz::sim::kNullEntity || joint.Entity() == gz::sim::kNullEntity)
        throw std::runtime_error("Body link or tail joint not found");
      body.EnableVelocityChecks(ecm);
      body.EnableAccelerationChecks(ecm);
      joint.EnablePositionCheck(ecm);
      joint.EnableVelocityCheck(ecm);
      dragLinks = {body, rotor[0], rotor[1], rotor[2]};
      dragTotalMass = 0;
      for (unsigned i = 0; i < dragLinks.size(); ++i)
      {
        const auto *inertial = ecm.Component<gz::sim::components::Inertial>(dragLinks[i].Entity());
        if (!inertial) throw std::runtime_error("Drag link lacks mass/CG");
        dragMass[i] = inertial->Data().MassMatrix().Mass();
        Positive(dragMass[i]);
        dragLocalCg[i] = inertial->Data().Pose().Pos();
        dragTotalMass += dragMass[i];
      }

      const auto three = [&sdf](const std::string &name, std::array<double, 3> fallback)
      {
        const auto v = sdf->Get<Vec>(name, Vec(fallback[0], fallback[1], fallback[2])).first;
        return std::array<double, 3> {v.X(), v.Y(), v.Z()};
      };
      calibration.pwm = three("pwm_knots_us", calibration.pwm);
      calibration.angleDeg = three("angle_knots_deg", calibration.angleDeg);
      calibration.adc = three("adc_knots", calibration.adc);
      calibration.Validate();
      maxThrust = three("max_thrust_n", maxThrust);
      reactionRatio = three("reaction_ratio_m", reactionRatio);
      spin = three("spin_sign", spin);
      rotorInertia = three("rotor_inertia_kgm2", rotorInertia);
      for (unsigned i = 0; i < 3; ++i)
      {
        Positive(maxThrust[i]); Nonnegative(reactionRatio[i]); Nonnegative(rotorInertia[i]);
        if (spin[i] != 1 && spin[i] != -1)
          throw std::runtime_error("spin_sign must contain only +1 or -1");
      }
      motorTau = sdf->Get<double>("motor_time_constant_s", motorTau).first;
      maxRotorSpeed = sdf->Get<double>("max_rotor_speed_rad_s", maxRotorSpeed).first;
      const auto motorModel = sdf->Get<std::string>("motor_model", "lag").first;
      if (motorModel != "lag" && motorModel != "dc_prop")
        throw std::runtime_error("motor_model must be lag or dc_prop");
      coupledMotor = motorModel == "dc_prop";
      if (coupledMotor)
      {
        MotorParameters parameters;
        parameters.kvRpmPerVolt = sdf->Get<double>("motor_kv_rpm_v", parameters.kvRpmPerVolt).first;
        parameters.resistanceOhm = sdf->Get<double>("motor_resistance_ohm", parameters.resistanceOhm).first;
        parameters.noLoadCurrentA = sdf->Get<double>("motor_no_load_current_a", parameters.noLoadCurrentA).first;
        parameters.noLoadVoltageV = sdf->Get<double>("motor_no_load_voltage_v", parameters.noLoadVoltageV).first;
        parameters.voltageV = sdf->Get<double>("motor_voltage_v", parameters.voltageV).first;
        parameters.currentLimitA = sdf->Get<double>("motor_current_limit_a", parameters.currentLimitA).first;
        parameters.brakeCurrentLimitA = sdf->Get<double>("motor_brake_current_limit_a", parameters.brakeCurrentLimitA).first;
        parameters.brakeOnStop = sdf->Get<bool>("motor_brake_on_stop", parameters.brakeOnStop).first;
        parameters.frictionSmoothingRadS = sdf->Get<double>("motor_friction_smoothing_rad_s", parameters.frictionSmoothingRadS).first;
        parameters.thrustCoefficientNPerRadS2 = sdf->Get<double>("prop_thrust_n_per_rad_s_squared", 0).first;
        parameters.torqueCoefficientNmPerRadS2 = sdf->Get<double>("prop_torque_nm_per_rad_s_squared", 0).first;
        Positive(parameters.thrustCoefficientNPerRadS2);
        Positive(parameters.torqueCoefficientNmPerRadS2);
        for (unsigned i = 0; i < 3; ++i)
        {
          motorParameters[i] = parameters;
          motorParameters[i].rotorInertiaKgM2 = rotorInertia[i];
          motorParameters[i].Validate();
        }
      }
      servoTau = sdf->Get<double>("servo_time_constant_s", servoTau).first;
      servoRate = sdf->Get<double>("servo_speed_deg_s", servoRate).first * kPi / 180;
      servoKp = sdf->Get<double>("servo_kp_nm_rad", servoKp).first;
      servoKd = sdf->Get<double>("servo_kd_nm_s_rad", servoKd).first;
      servoTorqueLimit = sdf->Get<double>("servo_max_torque_nm", servoTorqueLimit).first;
      watchdogSeconds = sdf->Get<double>("actuator_timeout_s", watchdogSeconds).first;
      linearDrag = sdf->Get<double>("linear_drag_n_s_m", linearDrag).first;
      const auto dragModel = sdf->Get<std::string>("airframe_drag_model", "quadratic_body").first;
      if (dragModel != "quadratic_body" && dragModel != "linear_world")
        throw std::runtime_error("airframe_drag_model must be quadratic_body or linear_world");
      quadraticDrag = dragModel == "quadratic_body";
      dragParameters.densityKgM3 = sdf->Get<double>("air_density_kg_m3", dragParameters.densityKgM3).first;
      dragParameters.cdaBodyM2 = three("drag_cda_body_m2", dragParameters.cdaBodyM2);
      dragParameters.Validate();
      angularDrag = sdf->Get<double>("angular_drag_nm_s_rad", angularDrag).first;
      homeLat = sdf->Get<double>("home_latitude_deg", homeLat).first;
      homeLon = sdf->Get<double>("home_longitude_deg", homeLon).first;
      traceRate = sdf->Get<double>("trace_rate_hz", traceRate).first;
      Positive(motorTau); Positive(maxRotorSpeed); Positive(servoTau); Positive(servoRate);
      Positive(servoKp); Nonnegative(servoKd); Positive(servoTorqueLimit);
      Positive(watchdogSeconds); Nonnegative(linearDrag); Nonnegative(angularDrag);
      Positive(traceRate);
      if (!std::isfinite(homeLat) || std::abs(homeLat) > 85 ||
          !std::isfinite(homeLon) || std::abs(homeLon) > 180)
        throw std::runtime_error("Invalid home coordinates");

      const int rawPort = sdf->Get<int>("actuator_port", 9001).first;
      const int fdmPort = sdf->Get<int>("fdm_port", 9003).first;
      const int adcPort = sdf->Get<int>("adc_port", 9005).first;
      for (const auto port : {rawPort, fdmPort, adcPort})
        if (port < 1024 || port > 65535) throw std::runtime_error("Invalid UDP port");
      socketFd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
      if (socketFd < 0) throw std::runtime_error("Cannot create UDP socket");
      const sockaddr_in local = Address(rawPort);
      if (bind(socketFd, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) != 0)
        throw std::runtime_error("Cannot bind actuator UDP port (already in use?)");
      fdmAddress = Address(fdmPort); adcAddress = Address(adcPort);

      std::string tracePath = sdf->Get<std::string>("trace_path", "").first;
      if (const char *env = std::getenv("TRICOPTER_TRACE")) tracePath = env;
      if (!tracePath.empty())
      {
        trace.open(tracePath, std::ios::out | std::ios::trunc);
        if (!trace) throw std::runtime_error("Cannot open trace CSV");
        trace << "sim_time_s,pwm0_us,pwm1_us,pwm2_us,servo_pwm_us,joint_angle_deg,"
          "joint_velocity_dps,servo_target_deg,pb0,pc0,thrust0_n,thrust1_n,thrust2_n,"
          "reaction0_nm,reaction1_nm,reaction2_nm,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s,roll_deg,pitch_deg,yaw_deg,"
          "gyro_frd_x,gyro_frd_y,gyro_frd_z,rear_axis_world_x,rear_axis_world_y,"
          "rear_axis_world_z,servo_torque_nm,actuator_valid";
        for (unsigned i = 0; i < 3; ++i)
        {
          trace << ",rotor" << i << "_omega_rad_s";
          for (const char *kind : {"spinup", "precession"})
            for (const char *axis : {"x", "y", "z"})
              trace << ",rotor" << i << '_' << kind << '_' << axis << "_nm";
        }
        for (unsigned i = 0; i < 3; ++i)
          trace << ",rotor" << i << "_terminal_current_a,rotor" << i << "_battery_current_a,rotor"
                << i << "_effective_voltage_v,rotor" << i << "_shaft_torque_nm,rotor"
                << i << "_prop_torque_nm,rotor" << i << "_numerical_dissipation_w";
        trace << ",airframe_drag_world_x_n,airframe_drag_world_y_n,airframe_drag_world_z_n,"
                 "airframe_drag_power_w,airframe_air_velocity_body_x_m_s,"
                 "airframe_air_velocity_body_y_m_s,airframe_air_velocity_body_z_m_s\n";
        trace << std::setprecision(12);
      }
      epoch = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
          Clock::now().time_since_epoch()).count());
      ResetState(); ready = true;
      gzmsg << "[TricopterBridge] Three motor PWM + servo0 UDP " << rawPort
            << "; FDM " << fdmPort << "; PB0-PC0 ADC " << adcPort << "\n";
    }
    catch (const std::exception &e)
    {
      gzerr << "[TricopterBridge] Configuration failed: " << e.what() << "\n";
      ready = false;
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (!ready) return;
    UpdateCameraHeading(ecm);
    const auto wallNow = Clock::now();
    const bool schedulingGap = havePreUpdateWallTime &&
      std::chrono::duration<double>(wallNow-lastPreUpdateWallTime).count() > watchdogSeconds;
    lastPreUpdateWallTime = wallNow; havePreUpdateWallTime = true;
    if (info.paused)
    {
      InvalidateActuators(); ReceiveActuators(); wasPaused = true; return;
    }
    if (wasPaused || schedulingGap) { InvalidateActuators(); wasPaused = false; }
    const double now = std::chrono::duration<double>(info.simTime).count();
    if (now < lastSimulationTime) ResetState();
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (!(dt > 0) || !std::isfinite(dt)) return;
    ReceiveActuators();
    actuatorValid = actuatorGate.CanDrive() &&
      std::chrono::duration<double>(Clock::now()-lastActuatorWallTime).count() <= watchdogSeconds;
    const auto position = joint.Position(ecm);
    const auto velocity = joint.Velocity(ecm);
    const double angle = position && !position->empty() ? position->at(0) : 0;
    const double speed = velocity && !velocity->empty() ? velocity->at(0) : 0;
    if (actuatorValid && actuator.servoPwm > 0)
    {
      const double requestedAngle = calibration.TargetRadians(actuator.servoPwm);
      servoTarget = ServoTargetStep(servoTarget, requestedAngle, dt, servoTau, servoRate);
      servoTorque = Clamp(servoKp*(servoTarget-angle) - servoKd*speed,
                           -servoTorqueLimit, servoTorqueLimit);
    }
    else
    {
      // Zero PWM means electrically suppressed, not a fabricated center pulse.
      servoTarget = angle; servoTorque = 0;
    }
    joint.SetForce(ecm, {servoTorque});

    for (unsigned i = 0; i < 3; ++i)
    {
      const double target = actuatorValid ? Clamp((actuator.motorPwm[i]-1000)/1000, 0, 1) : 0;
      const double previousSpeed = rotorSpeed[i];
      if (coupledMotor)
      {
        motorElectrical[i] = StepMotor(motorParameters[i], previousSpeed, target, dt,
                                      actuatorValid && actuator.motorPwm[i] > 0);
        rotorSpeed[i] = motorElectrical[i].omegaRadS;
        thrust[i] = motorElectrical[i].thrustN;
        reaction[i] = -spin[i] * motorElectrical[i].propTorqueNm;
      }
      else
      {
        motorState[i] = LagStep(motorState[i], target, dt, motorTau);
        rotorSpeed[i] = motorState[i] * maxRotorSpeed;
        thrust[i] = maxThrust[i] * motorState[i] * motorState[i];
        reaction[i] = -spin[i] * reactionRatio[i] * thrust[i];
      }
      const double omega = rotorSpeed[i];
      const auto pose = gz::sim::worldPose(rotor[i].Entity(), ecm);
      const Vec axis = pose.Rot().RotateVector(Vec::UnitZ);
      const Vec linkOmega = rotor[i].WorldAngularVelocity(ecm).value_or(Vec::Zero);
      // Optional rotor-relative angular momentum. SDF carrier inertia must
      // include the corresponding locked-rotor inertia when J is nonzero.
      rotorTorque[i] = RotorReaction(rotorInertia[i], spin[i]*omega,
        spin[i]*(omega-previousSpeed)/dt, {axis.X(), axis.Y(), axis.Z()},
        {linkOmega.X(), linkOmega.Y(), linkOmega.Z()});
      const auto sum = rotorTorque[i].Total();
      const Vec inertial(sum[0], sum[1], sum[2]);
      rotor[i].AddWorldWrench(ecm, thrust[i]*axis, reaction[i]*axis + inertial);
    }
    const auto bodyPose = gz::sim::worldPose(body.Entity(), ecm);
    if (quadraticDrag)
    {
      // Mass-weighted link-CG velocity includes the moving basket. Distributing
      // the force in the same mass fractions makes its resultant pass through
      // the instantaneous aircraft CG, with exactly F.v_CG mechanical power.
      Vec cgVelocity = Vec::Zero;
      for (unsigned i = 0; i < dragLinks.size(); ++i)
        cgVelocity += dragMass[i]/dragTotalMass *
          dragLinks[i].WorldLinearVelocity(ecm, dragLocalCg[i]).value_or(Vec::Zero);
      std::array<Vector, 3> axes;
      for (unsigned i = 0; i < 3; ++i)
      {
        Vec axis = Vec::Zero; axis[i] = 1;
        axis = bodyPose.Rot().RotateVector(axis);
        axes[i] = {axis.X(), axis.Y(), axis.Z()};
      }
      airframeDrag = QuadraticAirframeDrag(dragParameters,
        {cgVelocity.X(), cgVelocity.Y(), cgVelocity.Z()}, axes);
      const auto &force = airframeDrag.forceWorldN;
      const Vec drag(force[0], force[1], force[2]);
      for (unsigned i = 0; i < dragLinks.size(); ++i)
        dragLinks[i].AddWorldForce(ecm, dragMass[i]/dragTotalMass * drag);
      body.AddWorldWrench(ecm, Vec::Zero,
        -angularDrag * body.WorldAngularVelocity(ecm).value_or(Vec::Zero));
    }
    else
    {
      // Exact former model/application point retained for historical replays.
      const Vec velocity = body.WorldLinearVelocity(ecm).value_or(Vec::Zero);
      const Vec force = -linearDrag * velocity;
      const Vec bodyVelocity = bodyPose.Rot().RotateVectorReverse(velocity);
      airframeDrag = AirframeDragResult {};
      airframeDrag.forceWorldN = {force.X(), force.Y(), force.Z()};
      airframeDrag.velocityBodyMPS = {bodyVelocity.X(), bodyVelocity.Y(), bodyVelocity.Z()};
      airframeDrag.powerW = force.Dot(velocity);
      body.AddWorldWrench(ecm, force,
        -angularDrag * body.WorldAngularVelocity(ecm).value_or(Vec::Zero));
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override
  {
    if (!ready || info.paused) return;
    const double now = std::chrono::duration<double>(info.simTime).count();
    if (now <= lastSimulationTime) return; // Paused/repeated stamps never refresh ADC age.
    const auto pose = gz::sim::worldPose(body.Entity(), ecm);
    const auto rearPose = gz::sim::worldPose(rotor[0].Entity(), ecm);
    const Vec worldVelocity = body.WorldLinearVelocity(ecm).value_or(Vec::Zero);
    const Vec worldOmega = body.WorldAngularVelocity(ecm).value_or(Vec::Zero);
    const Vec worldAcceleration = body.WorldLinearAcceleration(ecm).value_or(Vec::Zero);
    const Vec specificFlu = pose.Rot().RotateVectorReverse(worldAcceleration - Vec(0, 0, -kGravity));
    const Vec omegaFlu = pose.Rot().RotateVectorReverse(worldOmega);
    const Vector gyro = BodyFluToSensorFrd({omegaFlu.X(), omegaFlu.Y(), omegaFlu.Z()});
    const Vector accel = FdmAccelerationFromSpecificFlu({specificFlu.X(), specificFlu.Y(), specificFlu.Z()});
    const auto q = FdmQuaternion({pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z()});
    // Native SITL's existing Gazebo GPS path mirrors the first latitude / longitude.
    // Mirror here too so its result is normal ENU geographic motion.
    const double latitude = homeLat - pose.Pos().Y()/111319.49;
    const double longitude = homeLon - pose.Pos().X()/(111319.49*std::cos(homeLat*kPi/180));
    const std::array<double, 18> fields {now,
      gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2],
      q[0], q[1], q[2], q[3], worldVelocity.X(), worldVelocity.Y(), worldVelocity.Z(),
      longitude, latitude, pose.Pos().Z(),
      101325*std::pow(1-2.25577e-5*pose.Pos().Z(), 5.25588)};
    const auto position = joint.Position(ecm);
    const auto velocity = joint.Velocity(ecm);
    if (!position || position->empty() || !std::isfinite(position->at(0))) return;
    const double jointAngle = position->at(0);
    const double jointSpeed = velocity && !velocity->empty() ? velocity->at(0) : 0;
    const uint16_t pb0 = calibration.PositionAdc(jointAngle);
    const auto adc = EncodeAdc(now, epoch, ++sequence, pb0);
    const auto fdm = EncodeFdm(fields);
    // Discard packets that arrived between PreUpdate's boundary drain and this
    // state send. No actuation is eligible until a post-state packet is read.
    if (!actuatorGate.IsAccepting()) ReceiveActuators();
    Send(adc.data(), adc.size(), adcAddress);
    const double wallNow = std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
    if (fdmPacer.CanSend(wallNow, now) && Send(fdm.data(), fdm.size(), fdmAddress))
    {
      fdmPacer.Sent(wallNow, now); actuatorGate.StateSent();
    }
    if (trace && now >= nextTraceTime)
    {
      const Vec euler = pose.Rot().Euler() * (180/kPi);
      const Vec rearAxis = rearPose.Rot().RotateVector(Vec::UnitZ);
      trace << now;
      for (double v : actuator.motorPwm) trace << ',' << v;
      trace << ',' << actuator.servoPwm << ',' << jointAngle*180/kPi << ','
        << jointSpeed*180/kPi << ',' << servoTarget*180/kPi << ',' << pb0 << ",0";
      for (double v : thrust) trace << ',' << v;
      for (double v : reaction) trace << ',' << v;
      trace << ',' << pose.Pos().X() << ',' << pose.Pos().Y() << ',' << pose.Pos().Z()
        << ',' << worldVelocity.X() << ',' << worldVelocity.Y() << ',' << worldVelocity.Z()
        << ',' << euler.X() << ',' << euler.Y() << ',' << euler.Z();
      for (double v : gyro) trace << ',' << v;
      trace << ',' << rearAxis.X() << ',' << rearAxis.Y() << ',' << rearAxis.Z()
        << ',' << servoTorque << ',' << actuatorValid;
      for (unsigned i = 0; i < 3; ++i)
      {
        trace << ',' << rotorSpeed[i];
        for (double value : rotorTorque[i].spinup) trace << ',' << value;
        for (double value : rotorTorque[i].precession) trace << ',' << value;
      }
      for (const auto &point : motorElectrical)
        trace << ',' << point.terminalCurrentA << ',' << point.batteryCurrentA << ','
              << point.effectiveVoltageV << ',' << point.shaftTorqueNm << ','
              << point.propTorqueNm << ',' << point.numericalDissipationW;
      for (double value : airframeDrag.forceWorldN) trace << ',' << value;
      trace << ',' << airframeDrag.powerW;
      for (double value : airframeDrag.velocityBodyMPS) trace << ',' << value;
      trace << '\n';
      trace.flush(); // External simulation pilots need each current 100 Hz state.
      nextTraceTime = now + 1/traceRate;
    }
    lastSimulationTime = now;
  }

  void Reset(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) override
  {
    ResetState();
    if (ready) ReceiveActuators();
  }

private:
  void UpdateCameraHeading(gz::sim::EntityComponentManager &ecm)
  {
    // This optional, separate kinematic world model has no geometry/collisions.
    // Keep the physics and GUI poses coherent through the supported model pose
    // command path. It is not attached to the aircraft and transmits no loads.
    if (cameraHeadingName.empty() || worldEntity == gz::sim::kNullEntity) return;
    if (cameraHeadingEntity == gz::sim::kNullEntity || !ecm.HasEntity(cameraHeadingEntity))
      cameraHeadingEntity = ecm.EntityByComponents(gz::sim::components::Model(),
        gz::sim::components::Name(cameraHeadingName),
        gz::sim::components::ParentEntity(worldEntity),
        gz::sim::components::Static(false));
    if (cameraHeadingEntity == gz::sim::kNullEntity) return;
    const auto bodyPose = gz::sim::worldPose(body.Entity(), ecm);
    const gz::math::Pose3d headingPose(bodyPose.Pos(),
      gz::math::Quaterniond(0, 0, bodyPose.Rot().Yaw()));
    gz::sim::Model(cameraHeadingEntity).SetWorldPoseCmd(ecm, headingPose);
    if (ecm.SetComponentData<gz::sim::components::Pose>(cameraHeadingEntity, headingPose))
      ecm.SetChanged(cameraHeadingEntity, gz::sim::components::Pose::typeId,
        // Motion is periodic, just like the aircraft's physics poses. Marking
        // this one-time would force SceneBroadcaster to publish every 1 ms and
        // overload the GUI state queue instead of using its normal 60 Hz rate.
        gz::sim::ComponentState::PeriodicChange);
  }
  static void Positive(double x)
  {
    if (!std::isfinite(x) || x <= 0) throw std::runtime_error("Expected positive coefficient");
  }
  static void Nonnegative(double x)
  {
    if (!std::isfinite(x) || x < 0) throw std::runtime_error("Expected nonnegative coefficient");
  }
  static sockaddr_in Address(int port)
  {
    sockaddr_in a {}; a.sin_family = AF_INET; a.sin_port = htons(static_cast<uint16_t>(port));
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK); return a;
  }
  bool Send(const uint8_t *data, std::size_t size, const sockaddr_in &destination)
  {
    // All endpoints are loopback; a missing peer cannot block the physics thread.
    const auto sent = sendto(socketFd, data, size, 0,
      reinterpret_cast<const sockaddr *>(&destination), sizeof(destination));
    if (sent != static_cast<ssize_t>(size) && !warnedSend)
    {
      gzerr << "[TricopterBridge] UDP send failed\n"; warnedSend = true;
    }
    return sent == static_cast<ssize_t>(size);
  }
  void InvalidateActuators()
  {
    actuatorGate.Invalidate(); actuator = Actuators {}; actuatorValid = false;
    fdmPacer.Reset();
    lastActuatorWallTime = Clock::time_point {};
  }
  void ReceiveActuators()
  {
    DrainActuatorQueue(actuatorGate,
      [this](uint8_t *data, std::size_t size) -> ssize_t
      {
        const auto n = recv(socketFd, data, size, 0);
        if (n >= 0) return n;
        return errno == EAGAIN || errno == EWOULDBLOCK ? -1 : -2;
      },
      [this](const Actuators &candidate)
      {
        actuator = candidate; lastActuatorWallTime = Clock::now();
        fdmPacer.RawReply(std::chrono::duration<double>(lastActuatorWallTime.time_since_epoch()).count());
      });
  }
  void ResetState()
  {
    ++epoch; sequence = 0; lastSimulationTime = -1; nextTraceTime = 0;
    InvalidateActuators(); wasPaused = false; havePreUpdateWallTime = false;
    motorState.fill(0); thrust.fill(0); reaction.fill(0);
    rotorSpeed.fill(0); rotorTorque.fill(RotorInertialTorque {}); motorElectrical.fill(MotorStep {});
    servoTarget = servoTorque = 0;
    airframeDrag = AirframeDragResult {};
  }

  bool ready {false}, actuatorValid {false}, warnedSend {false};
  bool wasPaused {false}, havePreUpdateWallTime {false};
  int socketFd {-1}; sockaddr_in fdmAddress {}, adcAddress {};
  gz::sim::Link body; std::array<gz::sim::Link, 3> rotor; gz::sim::Joint joint;
  std::array<gz::sim::Link, 4> dragLinks;
  std::array<Vec, 4> dragLocalCg;
  std::array<double, 4> dragMass {};
  double dragTotalMass {0};
  bool quadraticDrag {true};
  AirframeDragParameters dragParameters;
  AirframeDragResult airframeDrag;
  std::string cameraHeadingName;
  gz::sim::Entity worldEntity {gz::sim::kNullEntity}, cameraHeadingEntity {gz::sim::kNullEntity};
  Calibration calibration; Actuators actuator; ActuatorGate actuatorGate; FdmPacer fdmPacer;
  std::array<double, 3> maxThrust {15, 15, 15}, reactionRatio {.02, .02, .02};
  std::array<double, 3> spin {1, -1, 1}, rotorInertia {0, 0, 0};
  std::array<double, 3> motorState {}, thrust {}, reaction {};
  std::array<double, 3> rotorSpeed {};
  std::array<RotorInertialTorque, 3> rotorTorque {};
  std::array<MotorParameters, 3> motorParameters {};
  std::array<MotorStep, 3> motorElectrical {};
  bool coupledMotor {false};
  double motorTau {.03}, maxRotorSpeed {2000}, servoTau {.02}, servoRate {362};
  double servoKp {1.5}, servoKd {.03}, servoTorqueLimit {.45}, servoTarget {0}, servoTorque {0};
  double linearDrag {.15}, angularDrag {.02}, watchdogSeconds {.25};
  double homeLat {47.397742}, homeLon {8.545594}, traceRate {100};
  double lastSimulationTime {-1}, nextTraceTime {0};
  uint32_t epoch {0}, sequence {0}; Clock::time_point lastActuatorWallTime {}, lastPreUpdateWallTime {};
  std::ofstream trace;
};
}

GZ_ADD_PLUGIN(tricopter::TricopterBridge, gz::sim::System,
    tricopter::TricopterBridge::ISystemConfigure,
    tricopter::TricopterBridge::ISystemPreUpdate,
    tricopter::TricopterBridge::ISystemPostUpdate,
    tricopter::TricopterBridge::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(tricopter::TricopterBridge, "tricopter::TricopterBridge")
