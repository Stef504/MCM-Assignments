function [Inverse_Jacobian]= Pseudo_Inverse(Jacobian)

    %SVD of Jacobian
    [U,S,V]= svd(Jacobian);
    
    S_inv= zeros(size(S'));
    sing_vals= diag(S);
    sigma_min=min(sing_vals);
    threshold= 0.001;
    lamda=0.01;
    
    if sigma_min > threshold
        lamda_sq=0;
    end
    
    if sigma_min <= threshold
        lamda_sq= (1-(sigma_min/threshold)^2)*lamda;
    end
    
    for k= 1:length(sing_vals)
        sigma = sing_vals(k);
        S_inv(k,k)=sigma/(sigma^2+lamda_sq);
    
    end

    Inverse_Jacobian= V*S_inv*U';
    
end