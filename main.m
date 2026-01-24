function main()
%Add path
addpath('./simulation_scripts');
addpath('./tools')
addpath('./icat')
addpath('./tasks')
clc;clear;close all;
%Simulation Parameters
dt = 0.005;
end_time = 30;

% Initialize Franka Emika Panda Model
model = load("panda.mat");

%Simulation Setup
real_robot = false;

%Initiliaze panda_arm() Class, specifying the base offset w.r.t World Frame
arm1=panda_arm(model,eye(4));
wTb2 = [-1 0 0 1.06;
    0 -1 0 -0.01;
    0 0 1 0;
    0 0 0 1];
arm2=panda_arm(model,wTb2);

%Initialize Bimanual Simulator Class
coop_sim=cooperative_sim(dt,arm1,arm2,end_time);

%Define Object Shape and origin Frame
obj_length = 0.06;
w_obj_pos = [0.5 0 0.59]';
w_obj_ori = rotation(0,0,0);

%Set goal frames for left and right arm, based on object frame
%TODO: Set arm goal frame based on object frame.
arm1.setGoal(w_obj_pos, w_obj_ori, w_obj_pos - [obj_length/2; 0; 0],arm1.wTt(1:3, 1:3) * rotation(0, deg2rad(20), 0));
arm2.setGoal(w_obj_pos, w_obj_ori, w_obj_pos + [obj_length/2; 0; 0],arm2.wTt(1:3, 1:3) * rotation(0, deg2rad(20), 0));

%Define Object goal frame (Cooperative Motion)
% wTog=[arm1.wTt(1:3, 1:3) * rotation(0.0, deg2rad(30), 0.0) [0.65, -0.35, 0.28]'; 0 0 0 1]; % aggiungo la rotazione di 30 gradi perché poi l'errore lo calcolo in generale
wTog=[rotation(0.0, 0.0, 0.0) [0.65, -0.35, 0.28]'; 0 0 0 1];
arm1.set_obj_goal(wTog);
arm2.set_obj_goal(wTog);

%Define Tasks, input values(Robot type(L,R,BM), Task Name)
left_tool_task=tool_task("L","LT",false);
right_tool_task=tool_task("R","RT",false);
left_minimun_altitude_task=minimum_altitude_task("L","LMA",false);
right_minimun_altitude_task=minimum_altitude_task("R","RMA",false);
left_joint_limit_task=joint_limit_task("L","LJL",false);
right_joint_limit_task=joint_limit_task("R","RJL",false);
bim_rigid_constraint_task = bimanual_rigid_constraint_task("R","BRC",true);
left_move_object_task = move_object_task("L", "LMO", false);
right_move_object_task = move_object_task("R", "RMO", false);
stp_joints_task = stop_joints_task("R","SJ",false);

left_task_list = {left_tool_task, left_minimun_altitude_task, left_joint_limit_task, bim_rigid_constraint_task, left_move_object_task, stp_joints_task};
left_task_list_name = ["LTT", "LMAT", "LJLT", "BRCT", "LMOT", "SJT"];

right_task_list = {right_tool_task, right_minimun_altitude_task, right_joint_limit_task, bim_rigid_constraint_task, right_move_object_task, stp_joints_task};
right_task_list_name = ["RTT", "RMAT", "RJLT", "BRCT", "RMOT", "SJT"];;


%Actions for each phase: go to phase, coop_motion phase, end_motion phase
% TODO: add cooperation task
left_move_to = ["LJLT", "LMAT", "LTT"];
left_move_obj = ["LJLT", "LMAT", "LMOT"];
left_stop = ["LMAT", "SJT"];

right_move_to = ["RJLT", "RMAT", "RTT"];
right_move_obj = ["RJLT", "RMAT", "RMOT"];
right_stop = ["RMAT", "SJT"];

%Load Action Manager Class and load actions
leftActionManager = ActionManager();
rightActionManager = ActionManager();

leftActionManager.setTaskList(left_task_list, left_task_list_name);
rightActionManager.setTaskList(right_task_list, right_task_list_name);

leftActionManager.addAction(left_move_to, "LMT");
leftActionManager.addAction(left_move_obj, "LMO");
leftActionManager.addAction(left_stop, "LST");

rightActionManager.addAction(right_move_to, "RMT");
rightActionManager.addAction(right_move_obj, "RMO");
rightActionManager.addAction(right_stop, "RST");

leftActionManager.setCurrentAction("LMT", coop_sim.time);
rightActionManager.setCurrentAction("RMT", coop_sim.time);

%Initiliaze robot interface
robot_udp=UDP_interface(real_robot);

%Initialize logger
% logger=SimulationLogger(ceil(end_time/dt)+1,coop_sim,actionManager);

%Main simulation Loop
for t = 0:dt:end_time
    % 1. Receive UDP packets - DO NOT EDIT
    [ql,qr]=robot_udp.udp_receive(t);
    if real_robot==true %Only in real setup, assign current robot configuration as initial configuratio
        coop_sim.left_arm.q=ql;
        coop_sim.right_arm.q=qr;
    end
    % 2. Update Full kinematics of the bimanual system
    coop_sim.update_full_kinematics();

    % 3. Compute control commands for current action
    [q_dot_l]=leftActionManager.computeICAT(coop_sim,coop_sim.time);
    [q_dot_r]=rightActionManager.computeICAT(coop_sim,coop_sim.time);

    % 4. Step the simulator (integrate velocities)
    coop_sim.sim(q_dot_l, q_dot_r);

    % 5. Send updated state to Pybullet
    robot_udp.send(t,coop_sim)

    % TODO: modifica la condizione tenendo conto di entrambe le braccia

    % [v_ang, v_lin] = CartError(coop_sim.left_arm.wTg , coop_sim.left_arm.wTt);

    % if norm(v_lin) < 0.1 && actionManager.current_action == 1 && norm(v_ang) < 0.1
    %     actionManager.setCurrentAction("MO",  coop_sim.time);
    % end

    % r_toc = coop_sim.left_arm.wTo(1:3, 4) - coop_sim.left_arm.wTg(1:3,4); % <w>
    % tToc = [eye(3), coop_sim.left_arm.wTt(1:3, 1:3)' * r_toc; 0 0 0 1];
    % wToc = coop_sim.left_arm.wTt * tToc;
    % wTog = [coop_sim.left_arm.wTt(1:3, 1:3) * coop_sim.left_arm.wTog(1:3, 1:3), coop_sim.left_arm.wTog(1:3, 4); 0 0 0 1];

    % [v_ang2, v_lin2] = CartError(wTog ,wToc);

    % if norm(v_lin2) < 0.1 && actionManager.current_action == 2 && norm(v_ang2) < 0.1
    %     actionManager.setCurrentAction("ST",  coop_sim.time);
    % end

    % 6. Lggging
    % logger.update(coop_sim.time,coop_sim.loopCounter)
    coop_sim.time;
    % 7. Optional real-time slowdown
    SlowdownToRealtime(dt);

    % Display joint position and velocity, Display for a given action, a number
    % of tasks
    action=1;
    tasks=[1];
    % logger.plotAll(action,tasks);
end
