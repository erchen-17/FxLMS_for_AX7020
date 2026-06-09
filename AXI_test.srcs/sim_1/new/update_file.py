import sys
file_path = 'tb_dual_mic_fxlms_v2.v'

with open(file_path, 'rb') as f:
    raw_content = f.read()

# 1. input real yn; to input real yn; input real mu;
raw_content = raw_content.replace(b'input real yn;', b'input real yn;\n        input real mu;')

# 2. fwrite
raw_content = raw_content.replace(
    b'$fwrite(scv_file, "%.10f,%.10f,%.10f\\n", en, dn, yn);',
    b'$fwrite(scv_file, "%.10f,%.10f,%.10f,%.10f\\n", en, dn, yn, mu);'
)

# 3. compute_paths
raw_content = raw_content.replace(
    b'compute_paths(x_ref, y_ctrl, d_primary, s_secondary);',
    b'effective_mu_real = q16_to_norm($signed(dut.effective_mu));\n            compute_paths(x_ref, y_ctrl, d_primary, s_secondary);'
)

# 4. write_scv_data
raw_content = raw_content.replace(
    b'write_scv_data(e_mic, d_primary, y_ctrl);',
    b'write_scv_data(e_mic, d_primary, y_ctrl, effective_mu_real);'
)

with open(file_path, 'wb') as f:
    f.write(raw_content)

print('Done')