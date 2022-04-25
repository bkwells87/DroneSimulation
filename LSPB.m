function pos = LSPB(q0, qf, t, V, tf)
    tb = (q0-qf + V*tf)/V;
    a = V/tb;

    if (t > tf)
        t = tf;
    end

    if (t <= tb)
        pos = q0 + a/2*t^2;
    elseif t <= (tf-tb)
        pos = (qf+q0-V*tf)/2 + V*t;
    else
        pos = qf - a/2*tf^2 + a*tf*t - a/2*t^2;
    end
end