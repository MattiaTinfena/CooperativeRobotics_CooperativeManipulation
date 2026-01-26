classdef tool_speed_task < Task
    properties
        xt_dot;
    end

    methods
        function obj=tool_speed_task(robot_ID,taskID, smooth)
            obj.ID=robot_ID;
            obj.task_name=taskID;
            obj.smooth=smooth;
        end

        function updateReference(obj, robot_system)

            obj.xdotbar = obj.xt_dot(1:6);
        end

        function updateJacobian(obj,robot_system)

            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;
            end

            obj.J = robot.wJt;
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end