function main()
%Add path
addpath('./simulation_scripts');
addpath('./tools')
addpath('./icat')
addpath('./tasks')
clc;clear;close all;
%Simulation Parameters
dt = 0.005;
end_time = 20;

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
arm1.setGoal(w_obj_pos, w_obj_ori, w_obj_pos - [obj_length/2; 0; 0],arm1.wTt(1:3, 1:3) * rotation(0, deg2rad(20), 0));
arm2.setGoal(w_obj_pos, w_obj_ori, w_obj_pos + [obj_length/2; 0; 0],arm2.wTt(1:3, 1:3) * rotation(0, deg2rad(20), 0));

%Define Object goal frame (Cooperative Motion)
wTog=[rotation(0.0, 0.0, 0.0) [0.6, 0.4, 0.48]'; 0 0 0 1];
% wTog=[rotation(0.0, deg2rad(5), 0) [0.4, 0, 0]'; 0 0 0 1]; % To stress the system

arm1.set_obj_goal(wTog);
arm2.set_obj_goal(wTog);

%Define Tasks, input values(Robot type(L,R,BM), Task Name, smoothness)
left_tool_task=tool_task("L","LT",true);
right_tool_task=tool_task("R","RT",true);
left_minimun_altitude_task=minimum_altitude_task("L","LMA",true);
right_minimun_altitude_task=minimum_altitude_task("R","RMA",true);
left_joint_limit_task=joint_limit_task("L","LJL",true);
right_joint_limit_task=joint_limit_task("R","RJL",true);
left_tool_speed_task=tool_speed_task("L", "LTS", false);
right_tool_speed_task=tool_speed_task("R", "RTS", false);
left_move_object_task = move_object_task("L", "LMO", true);
right_move_object_task = move_object_task("R", "RMO", true);
left_stp_joints_task = stop_joints_task("L","LSJ",true);
right_stp_joints_task = stop_joints_task("R","RSJ",true);

left_task_list = {left_tool_speed_task, left_joint_limit_task, left_minimun_altitude_task, left_stp_joints_task, left_move_object_task, left_tool_task};
left_task_list_name = ["LTST", "LJLT", "LMAT", "LSJT", "LMOT", "LTT"];

right_task_list = {right_tool_speed_task, right_joint_limit_task, right_minimun_altitude_task, right_stp_joints_task, right_move_object_task, right_tool_task};
right_task_list_name = ["RTST", "RJLT", "RMAT", "RSJT", "RMOT", "RTT"];

%Actions for each phase: go to phase, coop_motion phase, end_motion phase
left_move_to = ["LJLT", "LMAT", "LTT"];
left_move_obj = ["LJLT", "LMAT", "LMOT"];
left_stop = ["LMAT", "LSJT"];

right_move_to = ["RJLT", "RMAT", "RTT"];
right_move_obj = ["RJLT", "RMAT", "RMOT"];
right_stop = ["RMAT", "RSJT"];

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

initial_time = coop_sim.time;

%Initiliaze robot interface
robot_udp=UDP_interface(real_robot);

%Initialize logger
leftLogger=SimulationLogger(ceil(end_time/dt)+1,coop_sim,leftActionManager);
rightLogger=SimulationLogger(ceil(end_time/dt)+1,coop_sim,rightActionManager);

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


    if(leftActionManager.current_action == 2 && rightActionManager.current_action == 2) || ...
            (leftActionManager.current_action == 3 && rightActionManager.current_action == 3)
        % Implementation of the Coordination Policy
        l_xt_dot_non_coop = coop_sim.left_arm.wJt * q_dot_l;
        r_xt_dot_non_coop = coop_sim.right_arm.wJt * q_dot_r;

        l_wToc = coop_sim.left_arm.wTt * coop_sim.left_arm.tTo;
        r_wToc = coop_sim.right_arm.wTt * coop_sim.right_arm.tTo;

        [l_v_ang, l_v_lin] = CartError(coop_sim.left_arm.wTog, l_wToc);
        [r_v_ang, r_v_lin] = CartError(coop_sim.right_arm.wTog, r_wToc);

        l_x_dot_des = 0.7 * [l_v_ang; l_v_lin];
        r_x_dot_des = 0.7 * [r_v_ang; r_v_lin];

        mu0 = 1e-4;
        mu_l = mu0 + norm(l_x_dot_des - l_xt_dot_non_coop);
        mu_r = mu0 + norm(r_x_dot_des - r_xt_dot_non_coop);

        Hl = coop_sim.left_arm.wJt * pinv(coop_sim.left_arm.wJt);
        Hr = coop_sim.right_arm.wJt * pinv(coop_sim.right_arm.wJt);

        xt_dot_coop = (1/(mu_l + mu_r)) * ((mu_l * l_xt_dot_non_coop) + (mu_r * r_xt_dot_non_coop));

        C = [Hl, -Hr];
        Hrl = [Hl, zeros(6); zeros(6), Hr];
        xt_dot_feas = Hrl * (eye(12)-(pinv(C) * C)) * [xt_dot_coop; xt_dot_coop];

        % Simulation of the communication to the 2 robots
        left_tool_speed_task.xt_dot = xt_dot_feas;
        right_tool_speed_task.xt_dot = xt_dot_feas;

        % Compute control commands for current action
        [q_dot_l]=leftActionManager.computeICAT(coop_sim,coop_sim.time, "LTST");
        [q_dot_r]=rightActionManager.computeICAT(coop_sim,coop_sim.time, "RTST");

    end

    % 4. Step the simulator (integrate velocities)
    coop_sim.sim(q_dot_l, q_dot_r);

    % 5. Send updated state to Pybullet
    robot_udp.send(t,coop_sim)

    % 6. Logging
    leftLogger.update(coop_sim.time,coop_sim.loopCounter)
    rightLogger.update(coop_sim.time,coop_sim.loopCounter)

    coop_sim.time;
    % 7. Optional real-time slowdown
    SlowdownToRealtime(dt);

    % 8. Action switching
    goal_reached = norm(coop_sim.left_arm.rot_to_goal) < 0.01 && norm(coop_sim.left_arm.dist_to_goal) < 0.001 && ...
        norm(coop_sim.right_arm.rot_to_goal) < 0.01 && norm(coop_sim.right_arm.dist_to_goal) < 0.001;
    goal_reached2 = norm(coop_sim.left_arm.rot_to_goal2) < 0.01 && norm(coop_sim.left_arm.dist_to_goal2) < 0.001 && ...
        norm(coop_sim.right_arm.rot_to_goal2) < 0.01 && norm(coop_sim.right_arm.dist_to_goal2) < 0.001;

    delta_time = coop_sim.time - initial_time;

    if leftActionManager.current_action == 1 && rightActionManager.current_action == 1 && goal_reached

        leftActionManager.setCurrentAction("LMO", coop_sim.time);
        rightActionManager.setCurrentAction("RMO", coop_sim.time);

        coop_sim.left_arm.tTo = pinv(coop_sim.left_arm.wTt) * coop_sim.left_arm.wTo;
        coop_sim.right_arm.tTo = pinv(coop_sim.right_arm.wTt) * coop_sim.right_arm.wTo;
        initial_time = coop_sim.time;

    elseif (leftActionManager.current_action == 2 && rightActionManager.current_action == 2 && goal_reached2) || (delta_time > 10)
        leftActionManager.setCurrentAction("LST", coop_sim.time);
        rightActionManager.setCurrentAction("RST", coop_sim.time);
        initial_time = coop_sim.time;
    end
end
% Display joint position and velocity, Display for a given action, a number of tasks

leftLogger.plotAll(0);
rightLogger.plotAll(500);