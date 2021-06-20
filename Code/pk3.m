function [tet0, tetd] = pk3(w, p, q, r, sigma)
 u = p-r; v=q-r;
 up = u-w*w'*u;
 vp = v-w*w'*v;
 sigmap2 = sigma^2-norm(w'*(p-q))^2;
 tet0 = atan2(w'*cross(up,vp),up'*vp);
 tetd = acos((norm(vp)^2+norm(up)^2-sigmap2)/(2*norm(up)*norm(vp)));
end