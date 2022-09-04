classdef RRT
    properties
        start
        goal
        min_rand
        max_rand
        expand_dis
        goal_sample_rate
        max_iter
        obstacle_list
        node_list
    end
    
    methods
        function obj = RRT(obstacleList, randArea, expandDis, goalSampleRate, maxIter)
            obj.start = nan;
            obj.goal = nan;
            obj.min_rand = randArea(1);
            obj.max_rand = randArea(2);
            obj.expand_dis = expandDis;
            obj.goal_sample_rate = goalSampleRate;
            obj.max_iter = maxIter;
            obj.obstacle_list = obstacleList;
            obj.node_list = nan;
        end
        
        function path = rrt_planning(obj,start, goal)
            obj.start = Node(start(1), start(2));
            obj.goal = Node(goal(1), goal(2));
            obj.node_list = [obj.start];
            path = [];
            
            for i = 1:obj.max_iter
                rnd = obj.sample();
                n_ind = obj.get_nearset_list_index(obj.node_list, rnd);
                nearestNode = obj.node_list(n_ind);
                
                theta = atan2(rnd(2) - nearestNode.y, rnd(1) - nearestNode.x);
                newNode = obj.get_new_node(theta, n_ind, nearestNode);
                noCollision = obj.check_segment_collision(newNode.x, newNode.y, nearestNode.x, nearestNode.y);
                if noCollision
                    [obj.node_list] = [obj.node_list; newNode];
                    
                    if obj.is_near_goal(newNode)
                        if obj.check_segment_collision(newNode.x, newNode.y, obj.goal.x, obj.goal.y)
                            [rows, ~] = size(obj.node_list);
                            lastIndex = rows;
                            
                            path = obj.get_final_course(lastIndex);
                            pathLen = obj.get_path_len(path);
                            return
                        end
                    end
                end
            end   
        end
        
        function rnd = sample(obj)
            if randi(100) > obj.goal_sample_rate
                rnd = [obj.min_rand + (obj.max_rand - obj.min_rand) * rand(), obj.min_rand + (obj.max_rand - obj.min_rand) * rand()];
            else
                rnd = [obj.goal.x, obj.goal.y];
            end
        end
        
        function newNode = choose_parent(obj, newNode, nearInds)
            if isempty(nearInds)
                return;
            end
            
            dList = [];
            for i = 1 : length(nearInds)
                dx = newNode.x - obj.node_list(nearInds(i)).x;
                dy = newNode.y - obj.node_list(nearInds(i)).y;
                d = norm([dx, dy]);
                theta = atan2(dy, dx);
                if obj.check_collision(obj.node_list(i, :), theta, d)
                    dList = [dList; obj.node_list(i, :).cost + d];
                else
                    dList = [dList; inf];
                end
            end
            
            [minCost, minCostIndex] = min(dList);
            minInd = nearInds(minCostIndex);
            
            if minCost == inf
                disp('min cost is inf');
                return
            end
            
            newNode.cost = minCost;
            newNode.parent = minInd;
        end
        

                     
        function Flag = check_segment_collision(obj, x1, y1, x2, y2)
            [rows, ~] = size(obj.obstacle_list);
            for i = 1:rows
                ox = obj.obstacle_list(i, 1); oy = obj.obstacle_list(i, 2); dsize = obj.obstacle_list(i, 3);
                v = [x1, y1];
                w = [x2, y2];
                p = [ox, oy];
                dd = obj.distance_squared_point_to_segment(v, w, p);
                if dd <= dsize^2
                    Flag = false;
                    return
                end
            Flag = true;
            end
        end
        
        function Flag = check_collision(obj, nearNode, theta, d)
            tmpNode = nearNode;
            end_x = tmpNode.x + math.cos(theta) * d;
            end_y = empNode.y + math.sin(theta) * d;
            Flag = obj.check_segment_collision(tmpNode.x, tmpNode.y, end_x, end_y);
        end
        
        function newNode = get_new_node(obj, theta, n_ind, nearestNode)
            newNode = nearestNode;
            newNode.x = newNode.x + obj.expand_dis * cos(theta);
            newNode.y = newNode.y + obj.expand_dis * sin(theta);
            
            newNode.cost = newNode.cost + obj.expand_dis;
            newNode.parent = n_ind;
        end
        
        function Flag = is_near_goal(obj, node)
            d = obj.line_cost(node, obj.goal);
            if d < obj.expand_dis
                Flag = true;
                return
            end
            Flag = false;
        end
        
        function path = get_final_course(obj, lastIndex)
            path = [obj.goal.x, obj.goal.y];
            while ~isempty(obj.node_list(lastIndex).parent)
                node = obj.node_list(lastIndex);
                path = [path; [node.x, node.y]];
                lastIndex = node.parent;
            end
            path = [path; [obj.start.x, obj.start.y]];
        end
    end
    
    methods(Static)
        function minIndex = get_nearset_list_index(nodes, rnd)
            [rows, ~] = size(nodes);
            dList = [];
            for i = 1:rows
                dList = [dList, (norm([nodes(i).x - rnd(1), nodes(i).y - rnd(2)]))^2];
            end
            [~, minIndex] = min(dList);
        end
        
        function d = line_cost(node1, node2)
            d = norm([node1.x - node2.x, node1.y - node2.y]);
        end
        
        function pathLen = get_path_len(path)
            pathLen = 0;
            [rows, ~] = size(path);
            for i = 2:rows
                node1_x = path(i, 1);
                node1_y = path(i, 2);
                node2_x = path(i-1, 1);
                node2_y = path(i-1, 2);
                pathLen = pathLen + norm([node1_x - node2_x, node1_y - node2_y]);
            end
        end
        
        function dd = distance_squared_point_to_segment(v, w, p)
            if v == w
                dd = dot(p - v, p - v);
                return
            end
            l2 = dot(w - v, w - v);
            t = max(0, min(1, dot(p - v, w - v) / l2));
            projection = v + t * (w - v);
            dd = dot(p - projection, p - projection);
        end
    end
end





















