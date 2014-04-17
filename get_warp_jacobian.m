function [JWi] = get_warp_jacobian(K, x, y, z)
    focals = [K(1,1); K(2,2)];
    
    JWi = [ 1/z, 0, -x/z^2, -(x*y)/z^2, 1+(x^2/z^2), -y/z; ...
            0, 1/z, -y/z^2, -(1+(y^2/z^2)), (x*y)/z^2, x/z];
    JWi = bsxfun(@times, JWi, focals);
end
