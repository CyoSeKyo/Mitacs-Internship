function Ans = Get_Street(IntsCoordinates, Street, x, y, m, StreetWidth) %find the corresponding street to the given point 
Ans = 0;                          %1 means "NULL"
ss = zeros(2);
tt = zeros(2);
for i = 2 : 2 : m              %For convenience, we only consider even indice
    ss = IntsCoordinates(Street(i - 1, 1), :);
    tt = IntsCoordinates(Street(i - 1, 2), :);
    if (i == 16)
        -1;
    end
    sx = ss(1); sy = ss(2); ex = tt(1); ey = tt(2);
    if (sx == ex)                 %consider the width of street
        sx = sx - StreetWidth(i, 1); ex = ex + StreetWidth(i, 2);
        sy = sy + StreetWidth(i, 3); ey = ey - StreetWidth(i, 4);
    else
        sx = sx + StreetWidth(i, 1); ex = ex - StreetWidth(i, 2);
        sy = sy - StreetWidth(i, 3); ey = ey + StreetWidth(i, 4);
    end
    if (x >= sx && x <= ex && y >= sy && y <= ey)
        Ans = i;
        break;
    end
end