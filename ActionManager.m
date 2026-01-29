classdef ActionManager < handle
    properties
        actions = {}
        actionsTest = {}
        actions_tag = {}
        actions_names = {}
        current_action = 1
        previous_action = 1
        action_changes = 0
        action_switch_time = {}
        all_task_list = {}
        all_task_names = []
        initial_time = 0
        tasks = {}
        ap_instructions = {}
    end

    methods

        function addAction(obj, taskStack, name)

            obj.actions_tag{end+1} = taskStack;
            [tf, idx] = ismember(taskStack, obj.all_task_names);
            idx = idx(tf);
            actionTasks = obj.all_task_list(idx);

            obj.actions{end+1} = actionTasks;
            obj.actions_names{end+1} = name;

        end


        function setTaskList(obj, task_list, task_names)
            obj.all_task_list = task_list;
            obj.all_task_names = task_names;
        end

        function [ydotbar] = computeICAT(obj, robot, time, additional_task)

            if (obj.action_changes == 0)
                for i = 1:length(obj.tasks)
                    obj.tasks{i}.ap = 1;
                end
            else
                if(length(obj.ap_instructions) == length(obj.tasks))
                    for i = 1:length(obj.tasks)
                        if(obj.ap_instructions(i) == 1)
                            if(obj.tasks{i}.smooth)
                                obj.tasks{i}.ap = IncreasingBellShapedFunction(obj.initial_time, obj.initial_time +2, 0, 1, time);
                            else
                                obj.tasks{i}.ap = 1;
                            end
                        elseif(obj.ap_instructions(i) == -1)
                            obj.tasks{i}.ap = DecreasingBellShapedFunction(obj.initial_time, obj.initial_time +2, 0, 1, time);
                        else
                            obj.tasks{i}.ap = 1;
                        end
                    end
                else
                    error("Error in computing ap_instructions")
                end

                % when gaussian transitory is ended
                if (time > obj.initial_time + 2)

                    obj.tasks = obj.actions{obj.current_action};
                    % disp(obj.tasks);
                    obj.action_changes = 0;
                    for i = 1:length(obj.tasks)
                        obj.tasks{i}.ap = 1;
                    end
                end
            end

            tasks_to_run = {};

            if (nargin == 4)

                coop_task_id = strcmp(obj.all_task_names, additional_task);

                if any(coop_task_id)

                    coop_task = obj.all_task_list{coop_task_id};
                    tasks_to_run = [{coop_task}, obj.tasks];
                    if obj.current_action == 2
                        coop_task.ap = 1;
                    elseif obj.current_action == 3
                        coop_task.ap = DecreasingBellShapedFunction(obj.initial_time, obj.initial_time +2, 0, 1, time);
                    end
                else
                    error("Not existing task");
                end

            else
                tasks_to_run = obj.tasks;
            end

            % 1. Update references, Jacobians, activations
            for i = 1:length(tasks_to_run)
                tasks_to_run{i}.updateReference(robot);
                tasks_to_run{i}.updateJacobian(robot);
                tasks_to_run{i}.updateActivation(robot);
            end

            % 2. Perform ICAT (task-priority inverse kinematics)
            ydotbar = zeros(7,1);
            Qp = eye(7);
            for i = 1:length(tasks_to_run)
                [Qp, ydotbar] = iCAT_task(tasks_to_run{i}.A * tasks_to_run{i}.ap, tasks_to_run{i}.J, ...
                    Qp, ydotbar, tasks_to_run{i}.xdotbar, ...
                    1e-4, 0.01, 10);
            end
            % 3. Last task: residual damping
            [~, ydotbar] = iCAT_task(eye(7), eye(7), Qp, ydotbar, zeros(7,1), 1e-4, 0.01, 10);
        end

        function setCurrentAction(obj, actionName, time)

            found = false;

            for i = 1:length(obj.actions_names)
                if strcmp(obj.actions_names{i}, actionName)
                    obj.previous_action = obj.current_action;
                    obj.current_action = i;
                    obj.action_changes = 1;
                    obj.initial_time = time;
                    found = true;
                    break;
                end
            end

            if ~found
                error('Action not found');
            end

            act_tags  = obj.actions_tag{obj.current_action};
            prev_tags = obj.actions_tag{obj.previous_action};


            union_tags = union(act_tags, prev_tags);
            mask_involved = ismember(obj.all_task_names, union_tags);
            all_tags = obj.all_task_names(mask_involved);

            [tf_all, idx_all] = ismember(all_tags, obj.all_task_names);
            obj.tasks = obj.all_task_list(idx_all(tf_all));

            in_act  = ismember(all_tags, act_tags);
            in_prev = ismember(all_tags, prev_tags);

            obj.ap_instructions = zeros(1, numel(all_tags));
            obj.ap_instructions( in_act &  in_prev) =  0;
            obj.ap_instructions( in_act & ~in_prev) = +1;
            obj.ap_instructions(~in_act &  in_prev) = -1;
        end
    end
end