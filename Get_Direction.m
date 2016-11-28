function Ans = Get_Direction(index, x1, x2, y1, y2)
Ans = 0;
difx = x1 - x2;
dify = y1 - y2;
if (index == 1)
    if (dify < 0) 
        Ans = 1;
    end
else
    if (difx < 0)
        Ans = 1;
    end
end


    