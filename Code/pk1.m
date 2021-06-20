function tet = pk1(w, p, q, r)
 u = p-r; v=q-r;
 up = u-w*w'*u;
 vp = v-w*w'*v;
 tet = atan2(w'*cross(up,vp),up'*vp);
end