function t = user_max_time(M1, M2, M3, M4)
    m1=max(max(M1));
    m2=max(max(M2));
    m3=max(max(M3));
    m4=max(max(M4));
    t=max(max(m1,m2),max(m3,m4));
end