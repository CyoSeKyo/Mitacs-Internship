function Ans = Get_InsStreet(InsP, DP1, DP2)
if (InsP(1) == DP1(1) && InsP(2) == DP1(2))
    DP = DP2;
else
    DP = DP1;
end
if (InsP(1) == DP(1))
    if (DP(2) > InsP(2))
        Ans = 2;
    else
        Ans = 4;
    end
else
    if (DP(1) > InsP(1))
        Ans = 3;
    else
        Ans = 1;
    end
end
