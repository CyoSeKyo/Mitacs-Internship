function Ans = Get_FlLocation(BgLo, EnLo, Next_Position);
Ans = zeros(1, 2);
diffx = EnLo(1) - BgLo(1);
diffy = EnLo(2) - BgLo(2);
if (abs(diffx) > abs(diffy))
    if (diffx > 0)
        Ans(1) = BgLo(1) + Next_Position;
        Ans(2) = BgLo(2);
    else
        Ans(1) = BgLo(1) - Next_Position;
        Ans(2) = BgLo(2);
    end
else
    if (diffy > 0)
        Ans(1) = BgLo(1);
        Ans(2) = BgLo(2) + Next_Position;
    else
        Ans(1) = BgLo(1);
        Ans(2) = BgLo(2) - Next_Position;
    end
end
