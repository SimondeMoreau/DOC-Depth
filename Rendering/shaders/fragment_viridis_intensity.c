    varying vec3 v_depth; 
    varying float v_intensity;

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

    float saturate( float x ) { return clamp( x, 0.0, 1.0 ); }

    vec3 viridis_quintic( float x )
    {
        x = saturate( x );
        vec4 x1 = vec4( 1.0, x, x * x, x * x * x ); // 1 x x2 x3
        vec4 x2 = x1 * x1.w * x; // x4 x5 x6 x7
        return vec3(
            dot( x1.xyzw, vec4( +0.280268003, -0.143510503, +2.225793877, -14.815088879 ) ) + dot( x2.xy, vec2( +25.212752309, -11.772589584 ) ),
            dot( x1.xyzw, vec4( -0.002117546, +1.617109353, -1.909305070, +2.701152864 ) ) + dot( x2.xy, vec2( -1.685288385, +0.178738871 ) ),
            dot( x1.xyzw, vec4( +0.300805501, +2.614650302, -12.019139090, +28.933559110 ) ) + dot( x2.xy, vec2( -33.491294770, +13.762053843 ) ) );
    }


    void main() {
        float u = 2.0 * gl_PointCoord.x - 1.0;
        float v = 2.0 * gl_PointCoord.y - 1.0;
        float sq_ruv = u*u + v*v;

        // Gen round points
        if (sq_ruv > 1.0) { discard; }

        gl_FragColor = vec4(viridis_quintic(v_intensity/80.0), 1.0); // Saturated intensity
        gl_FragDepth = v_depth.x;
    } 