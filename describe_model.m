function G_SS = describe_model(G)
    G_SS = canon(ss(G), 'modal');
    
    if isstable(G_SS) == true
        fprintf('\nSTABLE, ');
    else
        fprintf('\nUNSTABLE, ');
    end

    if rank(ctrb(G_SS)) == size(G_SS.A, 1)
        fprintf('CONTROLLABLE, ');
    else
        fprintf('UNCONTROLLABLE, ');
    end 

    if rank(obsv(G_SS)) == size(G_SS.A, 1)
        fprintf('OBSERVABLE\n');
    else
        fprintf('UNOBSERVABLE\n');
    end 
end