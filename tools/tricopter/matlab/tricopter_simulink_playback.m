function report = tricopter_simulink_playback(timeS, values, outputDirectory)
% A discrete SIGNAL REPLAY model, not an independent Simulink plant/controller.
assert(size(values,1) == numel(timeS) && numel(timeS) >= 2);
dt = timeS(2)-timeS(1);
assert(dt > 0 && max(abs(diff(timeS)-dt)) < 1e-9, 'Replay needs a uniform tick');
model = 'tricopter_offline_signal_replay';
assert(~bdIsLoaded(model), 'Replay model name is already loaded');
new_system(model);
cleanup = onCleanup(@() close_system(model,0)); %#ok<NASGU>
set_param(model,'Solver','FixedStepDiscrete','FixedStep',sprintf('%.17g',dt), ...
    'StartTime','0','StopTime',sprintf('%.17g',timeS(end)), ...
    'ReturnWorkspaceOutputs','on');
workspace = get_param(model,'ModelWorkspace');
assignin(workspace,'replaySignals',timeseries(values,timeS));
add_block('simulink/Sources/From Workspace',[model '/Synthetic trace'], ...
    'VariableName','replaySignals','Interpolate','off', ...
    'OutputAfterFinalValue','Holding final value','Position',[40 70 200 110]);
add_block('simulink/Sinks/To Workspace',[model '/Captured trace'], ...
    'VariableName','captured','SaveFormat','Structure With Time', ...
    'MaxDataPoints','inf','Position',[290 70 450 110]);
add_block('simulink/Sinks/Scope',[model '/Synthetic requested and actual response'], ...
    'Position',[300 160 450 200]);
add_line(model,'Synthetic trace/1','Captured trace/1');
add_line(model,'Synthetic trace/1','Synthetic requested and actual response/1');
annotation = Simulink.Annotation(model, ...
    'SIGNAL PLAYBACK ONLY: same-C synthetic request/actual trace; no independent plant or flight validation.');
annotation.Position = [40 10 620 45];
modelPath = fullfile(outputDirectory,[model '.slx']);
save_system(model,modelPath);
close_system(model,0);
load_system(modelPath); % Verify the saved model carries its own replay data.
result = sim(model);
captured = result.get('captured');
assert(numel(captured.time) == numel(timeS), 'Replay sample count differs');
assert(max(abs(captured.time(:)-timeS(:))) < 1e-9, 'Replay timestamps differ');
assert(isequal(size(captured.signals.values),size(values)), 'Replay signal shape differs');
error = max(abs(captured.signals.values-values),[],'all');
assert(error < 1e-10, 'Simulink changed the discrete replay values');
report = struct('executed',true,'passed',true,'rows',numel(timeS), ...
    'maximum_replay_difference',error,'interpolation',false, ...
    'scope','Discrete signal playback, not an independent dynamic plant');
end
