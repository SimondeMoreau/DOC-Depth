  attribute vec3 points;

  uniform mat4   T;    // Current LiDAR frame to Camera pose transformation
  uniform mat4   C;    // Extrinsic calibration
  uniform mat4   K; // Intrisinc calibration
  varying vec3 v_depth;
  
  uniform float max_pts_size;  
  uniform float min_pts_size;

  
  uniform float max_dist;
  void main() {
  
    if(abs(points.z) < 1 && abs(points.y)<1 && abs(points.x)<1){
        gl_Position = vec4(2,2,2,1);
        return;
    }
    vec4 pos =   C * T * vec4(points,1);
    float z = abs(pos.z);
    v_depth = vec3(z/max_dist,z/max_dist,z/max_dist);

    vec4 pos_proj = K * pos;

    gl_Position = vec4(pos_proj.x,pos_proj.y,pos_proj.z,pos_proj.w);

    gl_PointSize = clamp(max_pts_size / log(pos.x*pos.x + pos.y*pos.y + pos.z*pos.z), min_pts_size, max_pts_size);    

      
  } 