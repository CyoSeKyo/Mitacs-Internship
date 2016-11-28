function [Bg, Mid, En, Speed, In] = Calc_Property(Velocity, Position, LengthVP, width)

Speed = max(Velocity);

Bg = 0;
if (Position(1) < 40)
    while(Bg + 1 <= LengthVP && Velocity(Bg + 1) < Speed)
        Bg = Bg + 1;
    end
    if (Bg == 0)
        Bg = width / Speed;
    else
        Bg = Bg * 2;
    end
end

En = 0; In = 0;
temp = LengthVP + 1;
while(temp - 1 > 0 && Velocity(temp - 1) < Speed)
    temp = temp - 1;
end
if (temp < LengthVP + 1)
    while(temp <= LengthVP && Velocity(temp) > 0)
        temp = temp + 1;
        En = En + 2;
    end
    In = (LengthVP + 1 - temp) * 2;
else
    En = width / Speed;
end

Mid = 0;
if (Bg > 0 && En > 0)
    Mid = LengthVP * 2 - En - Bg - In;
end
