classdef Node 
    properties
        x
        y
        cost
        parent
    end
    
    methods
        function obj = Node(x,y)
            obj.x = x;
            obj.y = y;
        end
    end
end

