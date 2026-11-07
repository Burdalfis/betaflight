function report = validate_tricopter_dynamic_allocator(auditPath, outputDirectory)
% Independent bounded-LS and command-forward audit of sampled C candidates.
% This does not execute a second servo/airframe simulation.
assert(license('test','Optimization_Toolbox') == 1, 'Optimization Toolbox unavailable');
if ~isfolder(outputDirectory), mkdir(outputDirectory); end
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(fileparts(here)));
data = jsondecode(fileread(auditPath));
paths = cellstr(string(data.source_paths));
hashes = cellstr(string(data.source_sha256));
assert(numel(paths) == numel(hashes) && numel(unique(paths)) == numel(paths));
for index = 1:numel(paths)
    assert(strcmp(fileHash(fullfile(root,paths{index})),hashes{index}), ...
        'Native source differs from audit: %s',paths{index});
end
assert(strcmp(fileHash(fullfile(root,'tools','tricopter','dynamic_protocol.json')),data.protocol_sha256), ...
    'Dynamic protocol differs from frozen audit');
options = optimoptions('lsqlin','Algorithm','interior-point','Display','off', ...
    'OptimalityTolerance',1e-12,'ConstraintTolerance',1e-12, ...
    'StepTolerance',1e-12,'MaxIterations',1000);
snapshots = objects(data.snapshots);
records = {}; maximumEffortError = 0; maximumPrimaryError = 0;
maximumForwardCostError = 0; minimumExitFlag = Inf; checkedSnapshots = 0;
for row = 1:numel(snapshots)
    sample = snapshots{row};
    if ~sample.priority_available, continue; end
    checkedSnapshots = checkedSnapshots + 1;
    b = double(sample.effectiveness_matrix);
    scales = double(sample.response_scales(:)); net = double(sample.net_request(:));
    coefficient = double(sample.effort_per_omega_squared(:));
    maximum = double(sample.maximum_effort(:)); current = double(sample.current_rpm(:));
    assert(isequal(size(b),[4 4]) && numel(scales) == 4 && numel(net) == 4);
    assert(all(isfinite([b(:);scales;net;coefficient;maximum;current])) && ...
        all(scales > 0) && all(coefficient > 0) && all(maximum > 0));
    scalarInputs = [sample.horizon_us,sample.current_pwm_us,sample.minimum_pwm_us, ...
        sample.maximum_pwm_us,sample.high_cost_tolerance,sample.yaw_cost_tolerance, ...
        sample.movement_cost_tolerance,sample.minimum_high_cost,sample.minimum_yaw_cost];
    assert(all(isfinite(scalarInputs)) && sample.horizon_us > 0 && ...
        sample.maximum_pwm_us > sample.minimum_pwm_us && ...
        all(scalarInputs(5:9) >= 0), 'Malformed audit scalars');
    lower = coefficient .* (motorEndpoint(sample,current,sample.minimum_command_rpm(:))*pi/30).^2;
    upper = min(maximum,coefficient .* (motorEndpoint(sample,current,sample.maximum_command_rpm(:))*pi/30).^2);
    assert(all(lower <= upper), 'Empty reachable box');
    assert(max(abs(lower-sample.reachable_min_effort_exact_oracle(:))) < 1e-10);
    assert(max(abs(upper-sample.reachable_max_effort_exact_oracle(:))) < 1e-10);
    candidates = objects(sample.candidates); count = numel(candidates);
    primary = Inf(count,1); yaw = Inf(count,1); movement = Inf(count,1); valid = false(count,1);
    for index = 1:count
        candidate = candidates{index};
        if ~candidate.valid, continue; end
        valid(index) = true;
        theta = double(candidate.theta_h);
        a = [b(:,3)*cos(theta)+b(:,4)*sin(theta), b(:,1), b(:,2)];
        weighted = a(1:3,:)./scales(1:3);
        target = net(1:3)./scales(1:3);
        [effort,resnorm,~,exitFlag] = lsqlin(weighted,target,[],[],[],[],lower,upper,[],options);
        assert(exitFlag > 0, 'Independent lsqlin failed to solve a candidate');
        minimumExitFlag = min(minimumExitFlag,exitFlag);
        continuous = double(candidate.continuous_optimal_effort(:));
        assert(numel(continuous) == 3 && all(isfinite(continuous)), 'Malformed continuous effort');
        objective = 0.5*sum((weighted*continuous-target).^2);
        costError = abs(objective-0.5*resnorm);
        effortError = max(abs(continuous-effort));
        assert(costError <= 1e-6*max(1,0.5*resnorm), 'C primary objective differs from lsqlin');
        assert(effortError <= 1e-5*max(1,max(upper)), 'C primary solution differs from lsqlin');
        assert(all(continuous >= lower-2e-6) && all(continuous <= upper+2e-6));
        command = double(candidate.published_command_rpm(:));
        assert(all(command >= sample.minimum_command_rpm(:)) && all(command <= sample.maximum_command_rpm(:)));
        futureRpm = motorEndpoint(sample,current,command);
        futureEffort = coefficient .* (futureRpm*pi/30).^2;
        response = a*futureEffort; residual = (response-net)./scales;
        forward = [0.5*sum(residual(1:3).^2), 0.5*residual(4)^2, ...
            sum(((futureEffort-coefficient.*(current*pi/30).^2)./maximum).^2) + ...
            ((candidate.pwm_us-sample.current_pwm_us)/(sample.maximum_pwm_us-sample.minimum_pwm_us))^2];
        published = [candidate.published_high_cost,candidate.published_yaw_cost,candidate.published_movement_cost];
        assert(numel(published) == 3 && all(isfinite(published)) && all(published >= 0), ...
            'Published oracle costs must be finite and nonnegative');
        forwardError = max(abs(forward-published));
        assert(all(abs(forward-published) <= 2e-10+2e-9*abs(published)), 'Published-command costs differ');
        primary(index) = published(1); yaw(index) = published(2); movement(index) = published(3);
        maximumEffortError = max(maximumEffortError,effortError);
        maximumPrimaryError = max(maximumPrimaryError,costError);
        maximumForwardCostError = max(maximumForwardCostError,forwardError);
        records{end+1} = struct('snapshot',row,'candidate_index_zero_based',candidate.index, ... %#ok<AGROW>
            'lsqlin_exit_flag',exitFlag,'lsqlin_primary_cost',0.5*resnorm, ...
            'continuous_cost_difference',costError,'maximum_effort_difference',effortError, ...
            'published_forward_cost_difference',forwardError);
    end
    assert(any(valid), 'Priority available without a valid candidate');
    minimumHigh = min(primary(valid));
    highEligible = valid & primary <= minimumHigh+sample.high_cost_tolerance;
    minimumYaw = min(yaw(highEligible));
    yawEligible = highEligible & yaw <= minimumYaw+sample.yaw_cost_tolerance;
    minimumMove = min(movement(yawEligible));
    selected = find(yawEligible & movement <= minimumMove+sample.movement_cost_tolerance,1);
    assert(selected-1 == sample.selected_index, 'Global lexicographic selection differs');
    assert(minimumHigh == sample.minimum_high_cost && minimumYaw == sample.minimum_yaw_cost);
end
assert(checkedSnapshots > 0 && ~isempty(records), 'No valid candidate problems were audited');
for index = 1:numel(paths)
    assert(strcmp(fileHash(fullfile(root,paths{index})),hashes{index}), 'Source changed during MATLAB audit');
end
report = struct('passed',true,'matlab_version',version,'snapshots_checked',checkedSnapshots, ...
    'candidate_problems_checked',numel(records),'minimum_lsqlin_exit_flag',minimumExitFlag, ...
    'maximum_continuous_effort_difference',maximumEffortError, ...
    'maximum_continuous_primary_cost_difference',maximumPrimaryError, ...
    'maximum_published_forward_cost_difference',maximumForwardCostError, ...
    'global_priority_selection_checked',true,'calibrated',false,'for_flight',false, ...
    'audit_sha256',fileHash(auditPath),'protocol_sha256',data.protocol_sha256, ...
    'validator_sha256',fileHash([mfilename('fullpath') '.m']), ...
    'source_paths',{paths},'source_sha256',{hashes},'candidates',{records}, ...
    'scope','Independent motor/box/priority algebra audit; exported servo forecasts are inputs, not independently validated physics');
fid = fopen(fullfile(outputDirectory,'matlab-dynamic-validation.json'),'w');
assert(fid >= 0); cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
disp(jsonencode(rmfield(report,{'candidates','source_paths','source_sha256'})));
end

function output = motorEndpoint(sample,current,command)
output = zeros(3,1); dt = sample.horizon_us/1e6;
for motor = 1:3
    error = command(motor)-current(motor); distance = abs(error);
    if error >= 0
        tau = sample.tau_increasing_s(motor); rate = sample.slew_increasing_rpm_s(motor);
    else
        tau = sample.tau_decreasing_s(motor); rate = sample.slew_decreasing_rpm_s(motor);
    end
    assert(tau > 0 && rate >= 0 && isfinite(tau) && isfinite(rate));
    if rate == 0 || distance <= rate*tau
        remaining = distance*exp(-dt/tau);
    else
        linearTime = (distance-rate*tau)/rate;
        if dt <= linearTime, remaining = distance-rate*dt;
        else, remaining = rate*tau*exp(-(dt-linearTime)/tau); end
    end
    output(motor) = command(motor)-sign(error)*remaining;
end
end

function result = objects(value)
if iscell(value), result = value; else, result = num2cell(value); end
end

function result = fileHash(path)
fid = fopen(path,'rb'); assert(fid >= 0, 'Cannot read audit source: %s',path);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
digest = java.security.MessageDigest.getInstance('SHA-256'); digest.update(fread(fid,Inf,'*uint8'));
result = lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
