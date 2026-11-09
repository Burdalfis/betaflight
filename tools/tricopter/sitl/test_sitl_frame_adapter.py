"""Apply the actual common patch temporarily and test its native C consumers.

The fixture compiles the patched header and the actual patched Mahony-call,
ANGLE-coordination and FDM-quaternion blocks. Golden orientations come from
independent rigid-body quaternion integration, not duplicated adapter formulae.
The checkout's firmware sources are only read.
"""
from pathlib import Path
import hashlib
import subprocess
import tempfile
import unittest


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
PATCH = HERE/'patches/corrected-frame.patch'
PATHS = ('src/platform/SIMULATOR/sitl_gyro.h', 'src/main/flight/imu.c',
         'src/main/flight/pid.c', 'src/platform/SIMULATOR/sitl.c')
PREIMAGES = dict(zip(PATHS, (
    '6d906ee9e15cbcec7f1fe4d3997f183051f63db5054daad40bfa3f03c193132f',
    'b4ea5d546ab605cb9515d34c1d1b4461b06b9eacd552bd282c3835a503f24584',
    'd1065b12de3936c8bbc0385e8fee81928bb7f08779f3d0f915ab99f69b75e013',
    '630d629bc97e3c74de1de83b22b38378a82d5778c8779729809fc7244f276541')))


FIXTURE = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "sitl_gyro.h"
#define PI 3.14159265358979323846
#define DEGREES_TO_RADIANS(x) ((x)*(PI/180.0))
#define FLIGHT_MODE(x) (0)
#define sin_approx(x) sinf(x)
enum { X=0, Y=1, Z=2, XYZ_AXIS_COUNT=3, FD_ROLL=0, FD_PITCH=1 };
static float sourceGyro[3], estimatorGyro[3];
static struct { struct { float x,y,z; } accADC; } acc;
static float gyroGetFilteredDownsampled(int axis) { return sourceGyro[axis]; }
static bool imuIsAccelerometerHealthy(void) { return false; }
static float imuCalcKpGain(uint32_t now, bool useAcc, float rates[3]) { return 0; }
static void imuMahonyAHRSupdate(float dt,float gx,float gy,float gz,
    bool useAcc,float ax,float ay,float az,float magErr,float cogErr,float kp)
{ estimatorGyro[0]=gx; estimatorGyro[1]=gy; estimatorGyro[2]=gz; }
static void capture_estimator_arguments(const double pidGyro[3])
{
    for (int axis=0;axis<3;++axis) sourceGyro[axis]=(float)(pidGyro[axis]*180/PI);
    const float dt=.001f, magErr=0, cogErr=0; const uint32_t currentTimeUs=0;
    /* ACTUAL_IMU_BLOCK */
}
static float coordinated_rate(int axis,float roll,float pitch,float yaw,float gain)
{
    struct { float angleTarget[2],angleYawSetpoint,angleEarthRef; } pidRuntime={{roll,pitch},yaw,gain};
    float angleRate=0;
    /* ACTUAL_PID_BLOCK */
    return angleRate;
}
typedef struct { double w,x,y,z; } Q;
static Q multiply(Q a,Q b)
{ return (Q){a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
    a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
    a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
    a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w}; }
static Q normalize(Q q)
{ double n=sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z); return (Q){q.w/n,q.x/n,q.y/n,q.z/n}; }
static Q from_euler(double roll,double pitch,double yaw)
{
    Q x={cos(roll/2),sin(roll/2),0,0}, y={cos(pitch/2),0,sin(pitch/2),0}, z={cos(yaw/2),0,0,sin(yaw/2)};
    return multiply(multiply(z,y),x);
}
static void euler(Q input,double result[3])
{
    Q q=normalize(input);
    result[0]=atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y));
    double s=2*(q.w*q.y-q.x*q.z); s=fmax(-1,fmin(1,s)); result[1]=asin(s);
    result[2]=atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
static Q packet_to_attitude(Q packet)
{
    const struct { double imu_orientation_quat[4]; } value={{packet.w,packet.x,packet.y,packet.z}};
    const __typeof__(value) *pkt=&value;
    /* ACTUAL_QUATERNION_BLOCK */
    return (Q){attQw,attQx,attQy,attQz};
}
static Q world_to_packet(Q world) { return (Q){world.w,world.x,-world.y,-world.z}; }
static void check(bool okay,const char *what)
{ if(!okay) { fprintf(stderr,"FAIL: %s\n",what); exit(1); } }
static void near(double a,double b,double tolerance,const char *what)
{ if(fabs(a-b)>tolerance) { fprintf(stderr,"FAIL: %s got %.12g expected %.12g\n",what,a,b); exit(1); } }
static Q integrate(Q q,const double omega[3],double dt,int steps)
{
    for(int i=0;i<steps;++i) {
        Q dq=multiply(q,(Q){0,omega[0],omega[1],omega[2]});
        q=normalize((Q){q.w+.5*dt*dq.w,q.x+.5*dt*dq.x,q.y+.5*dt*dq.y,q.z+.5*dt*dq.z});
    }
    return q;
}
static void test_mapping_and_guards(void)
{
    const double packet[3]={1,2,3}; double pid[3];
    sitlGyroBodyFromSim(packet,true,&pid[0],&pid[1],&pid[2]);
    near(pid[0],1,0,"PID roll unchanged");near(pid[1],-2,0,"PID pitch unchanged");near(pid[2],3,0,"PID yaw unchanged");
    near(sitlGyroYawForAttitude(3,true),-3,0,"Gazebo attitude yaw");
    near(sitlGyroYawForAttitude(3,false),3,0,"legacy attitude yaw");
    capture_estimator_arguments(pid);
    near(estimatorGyro[0],1,1e-6,"Mahony X input");near(estimatorGyro[1],-2,1e-6,"Mahony Y input");
    near(estimatorGyro[2], ENABLE_SIMULATOR && ENABLE_GAZEBO_BRIDGE ? -3:3,1e-6,"guarded Mahony Z input");
    near(sourceGyro[2],(float)(3*180/PI),0,"rate PID sample never rewritten");
}
static void test_cardinal_headings(void)
{
    const double worldYaw[]={0,PI/2,PI,-PI/2}, expected[]={90,0,270,180};
    for(int i=0;i<4;++i) {
        Q world=from_euler(0,0,worldYaw[i]); double angles[3]; euler(packet_to_attitude(world_to_packet(world)),angles);
        double heading=fmod(-angles[2]*180/PI+360,360); if(fabs(heading-360)<1e-5)heading=0;
        near(heading,expected[i],1e-5,"ENU to NWU cardinal heading");
        near(angles[0],0,1e-6,"cardinal roll preserved");near(angles[1],0,1e-6,"cardinal pitch preserved");
    }
}
static void test_tilted_yaw_integration(void)
{
    const double tilts[][2]={{0,20},{0,-20},{20,0},{-20,0},{15,20},{-15,-20}};
    for(int k=0;k<6;++k) for(int direction=-1;direction<=1;direction+=2) {
        const double roll=tilts[k][0]*PI/180,pitch=tilts[k][1]*PI/180;
        const double yaw=.3, speed=direction*60*PI/180;
        Q world=from_euler(roll,pitch,yaw), inverse={world.w,-world.x,-world.y,-world.z};
        Q body=multiply(multiply(inverse,(Q){0,0,0,-speed}),world);
        const double packet[3]={body.x,-body.y,-body.z}; double pid[3];
        sitlGyroBodyFromSim(packet,true,&pid[0],&pid[1],&pid[2]);
        capture_estimator_arguments(pid);
        if(ENABLE_SIMULATOR && ENABLE_GAZEBO_BRIDGE) {
            const double omega[3]={estimatorGyro[0],estimatorGyro[1],estimatorGyro[2]};
            Q initial=packet_to_attitude(world_to_packet(world));
            Q integrated=integrate(initial,omega,.0005,2000);
            double actual[3]; euler(integrated,actual);
            near(actual[0],roll,4e-6,"tilted yaw must not create false roll");
            near(actual[1],pitch,4e-6,"tilted yaw must not create false pitch");
            Q expected=multiply(from_euler(0,0,-PI/2),from_euler(roll,pitch,yaw-speed));
            Q difference=multiply((Q){expected.w,-expected.x,-expected.y,-expected.z},integrated);
            near(fabs(difference.w),1,1e-10,"integrated full orientation matches world trajectory");
            Q old=integrate(initial,pid,.0005,2000); double bad[3];euler(old,bad);
            check(hypot(bad[0]-roll,bad[1]-pitch)>PI/180,"old unadapted yaw produces false Euler RP");
        }
    }
}
static void test_coordinated_turn(void)
{
    for(int sign=-1;sign<=1;sign+=2)for(int direction=-1;direction<=1;direction+=2) {
        double angle=sign*20, yaw=direction*60, sine=sin(angle*PI/180);
        double desiredRoll=yaw*sine,desiredPitch=-yaw*sine;
        double expectedSign=ENABLE_SIMULATOR && ENABLE_GAZEBO_BRIDGE ? 1:-1;
        near(coordinated_rate(FD_ROLL,0,angle,yaw,1),expectedSign*desiredRoll,2e-5,"pitched coordinated roll");
        near(coordinated_rate(FD_PITCH,angle,0,yaw,1),expectedSign*desiredPitch,2e-5,"rolled coordinated pitch");
        near(coordinated_rate(FD_ROLL,0,angle,yaw,0),0,0,"zero earth-reference gain");
        near(coordinated_rate(FD_PITCH,angle,0,yaw,0),0,0,"zero pitch coordination gain");
    }
}
int main(void)
{
    test_mapping_and_guards(); test_cardinal_headings();
    test_tilted_yaw_integration(); test_coordinated_turn();
    printf("frame adapter physics PASS simulator=%d gazebo=%d\n",ENABLE_SIMULATOR,ENABLE_GAZEBO_BRIDGE);
    return 0;
}
'''


class SitlFrameAdapter(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix='tricopter-frame-tests-')
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.tree = Path(cls.temporary.name)
        cls.before = {name:(ROOT/name).read_bytes() for name in PATHS}
        for name,contents in cls.before.items():
            if hashlib.sha256(contents).hexdigest() != PREIMAGES[name]:
                raise AssertionError('Firmware preimage drift: '+name)
            destination=cls.tree/name;destination.parent.mkdir(parents=True,exist_ok=True)
            destination.write_bytes(contents)
        subprocess.run(['git','apply','--check',str(PATCH)],cwd=cls.tree,check=True,capture_output=True,text=True)
        subprocess.run(['git','apply',str(PATCH)],cwd=cls.tree,check=True,capture_output=True,text=True)
        imu=(cls.tree/PATHS[1]).read_text()
        imu_start=imu.index('    float gyroAverage[XYZ_AXIS_COUNT];')
        imu_block=imu[imu_start:imu.index('    imuUpdateEulerAngles();',imu_start)]
        pid=(cls.tree/PATHS[2]).read_text()
        pid_start=pid.index('    float sinAngle = sin_approx(')
        pid_block=pid[pid_start:pid.index('    pidRuntime.angleTarget[axis] = angleTarget;',pid_start)]
        sitl=(cls.tree/PATHS[3]).read_text()
        quaternion_start=sitl.index('    const float pktQw = pkt->imu_orientation_quat[0];')
        quaternion_block=sitl[quaternion_start:sitl.index('#else',quaternion_start)]
        code=FIXTURE.replace('/* ACTUAL_IMU_BLOCK */',imu_block).replace('/* ACTUAL_PID_BLOCK */',pid_block)
        code=code.replace('/* ACTUAL_QUATERNION_BLOCK */',quaternion_block)
        cls.source=cls.tree/'frame_physics.c';cls.source.write_text(code)

    def compile_and_run(self, simulator, gazebo):
        binary=self.tree/f'physics-{simulator}-{gazebo}'
        result=subprocess.run(['cc','-std=gnu11','-O2','-Wall','-Wextra','-Werror','-Wno-unused-parameter',
            f'-DENABLE_SIMULATOR={simulator}',f'-DENABLE_GAZEBO_BRIDGE={gazebo}',
            '-I'+str(self.tree/'src/platform/SIMULATOR'),str(self.source),'-lm','-o',str(binary)],
            capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        result=subprocess.run([str(binary)],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    def test_gazebo_physics_cardinals_and_native_call_sites(self): self.compile_and_run(1,1)
    def test_legacy_bridge_behavior_preserved(self): self.compile_and_run(1,0)
    def test_hardware_branch_behavior_preserved(self): self.compile_and_run(0,0)

    def test_patch_scope_and_original_firmware_sources_unchanged(self):
        changed={line[6:] for line in PATCH.read_text().splitlines() if line.startswith('+++ b/')}
        self.assertEqual(changed,set(PATHS))
        for name,contents in self.before.items():
            self.assertEqual((ROOT/name).read_bytes(),contents,name)
            self.assertNotEqual((self.tree/name).read_bytes(),contents,name)


if __name__ == '__main__': unittest.main()
