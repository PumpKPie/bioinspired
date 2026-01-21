shader_type spatial;
render_mode unshaded, depth_test_disabled; // Makes points visible through walls if needed for debugging

uniform sampler2D point_data;
uniform int point_count;
uniform float tex_size = 512.0;

void vertex() {
    // 1. Hide unused instances
    if (INSTANCE_ID >= point_count) {
        MODEL_MATRIX[3] = vec4(0.0, -1000.0, 0.0, 1.0);
    } else {
        // 2. Fetch from 1D Texture
        float x = mod(float(INSTANCE_ID), tex_size);
        float y = floor(float(INSTANCE_ID) / tex_size);
        vec2 uv = vec2((x + 0.5) / tex_size, (y + 0.5) / tex_size);

        vec3 world_pos = texture(point_data, uv).rgb;

        // 3. Force world position into the Model Matrix
        // We use the 4th column (index 3) for X, Y, Z translation
        MODEL_MATRIX[3][0] = world_pos.x;
        MODEL_MATRIX[3][1] = world_pos.y;
        MODEL_MATRIX[3][2] = world_pos.z;
    }
}

void fragment() {
    ALBEDO = vec3(0.0, 1.0, 1.0); // Cyan
}