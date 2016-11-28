function Ans = Find_RealStr(Set, x)

len = length(Set);
for i = 1 : len
    if (Set(i) == x)
        Ans = i;
    end
end

