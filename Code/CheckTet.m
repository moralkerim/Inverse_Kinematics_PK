function tetd_c = CheckTet(tetd)
tetd_c = zeros(1,6);

TET1_LIM_M = -170;
TET1_LIM_P = +170;
TET2_LIM_M = -95;
TET2_LIM_P = +132;
TET3_LIM_M = -163;
TET3_LIM_P = +73;
TET4_LIM_M = -180;
TET4_LIM_P = +180;
TET5_LIM_M = -133;
TET5_LIM_P = +133;

TET_LIM_M = [TET1_LIM_M TET2_LIM_M TET3_LIM_M...
    TET4_LIM_M TET5_LIM_M];

TET_LIM_P = [TET1_LIM_P TET2_LIM_P TET3_LIM_P...
    TET4_LIM_P TET5_LIM_P];

for i=1:5
    if(tetd(i) >= 0)
        if(tetd(i) > TET_LIM_P(i))
            tetd_c(i) = tetd(i)-360;
            norm_new = abs(tetd_c(i)-TET_LIM_M(i));
            norm_old = abs(tetd(i)  -TET_LIM_P(i));
            if(norm_old < norm_new)
                tetd_c(i) = tetd(i);
            end
        else
            tetd_c(i) = tetd(i);
        end
    else
        if(tetd(i) < TET_LIM_M(i))
            tetd_c(i) = tetd(i)-360;
            norm_new = abs(tetd_c(i)-TET_LIM_P(i));
            norm_old = abs(tetd(i)  -TET_LIM_M(i));
            if(norm_old < norm_new)
                tetd_c(i) = tetd(i);
            end
        else
            tetd_c(i) = tetd(i);
        end
    end
    
end

tetd_c(6) = tetd(6);
end

