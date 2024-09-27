    varying vec3 v_depth; 

    vec4 turbo(float x) {
    float r = 0.1357 + x * ( 4.5974 - x * ( 42.3277 - x * ( 130.5887 - x * ( 150.5666 - x * 58.1375 ))));
    float g = 0.0914 + x * ( 2.1856 + x * ( 4.8052 - x * ( 14.0195 - x * ( 4.2109 + x * 2.7747 ))));
    float b = 0.1067 + x * ( 12.5925 - x * ( 60.1097 - x * ( 109.0745 - x * ( 88.5066 - x * 26.8183 ))));
    return vec4(r,g,b,1);
    }

    float colormap_red(float x) {
        if (x < 0.7) {
            return 4.0 * x - 1.5;
        } else {
            return -4.0 * x + 4.5;
        }
    }

    float colormap_green(float x) {
        if (x < 0.5) {
            return 4.0 * x - 0.5;
        } else {
            return -4.0 * x + 3.5;
        }
    }

    float colormap_blue(float x) {
        if (x < 0.3) {
        return 4.0 * x + 0.5;
        } else {
        return -4.0 * x + 2.5;
        }
    }

    vec4 colormap(float x) {
        float r = clamp(colormap_red(x), 0.0, 1.0);
        float g = clamp(colormap_green(x), 0.0, 1.0);
        float b = clamp(colormap_blue(x), 0.0, 1.0);
        return vec4(r, g, b, 1.0);
    }

    void main() {
        float u = 2.0 * gl_PointCoord.x - 1.0;
        float v = 2.0 * gl_PointCoord.y - 1.0;
        float sq_ruv = u*u + v*v;

        // Gen round points
        if (sq_ruv > 1.0) { discard; }
        
        gl_FragColor = turbo(v_depth.x);
        gl_FragDepth = v_depth.x;
    } 