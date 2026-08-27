# GRIT policy

This directory contains the GRIT deployment model and its runtime contract:

```text
checkpoints/
  policy.onnx
  policy.json
```

SHA-256 checksums:

```text
1f71c3d725d4d120160cdfd75e50b7da4eab2bd3b7876d7cb754d88b43b58082  policy.onnx
c76832c018f8b6dd631dcd3baaa0511f57485c5b3e2e8c069fd9bb5c5774cdc3  policy.json
```

The ONNX graph must expose the multi-input deployment interface used by the
runtime:

- input `reference_context`: `[batch, 9, 70]`
- input `proprio_history`: `[batch, 990]`
- output `actions`: `[batch, 29]`

The runtime validates the ONNX input and output names directly and rejects an
incompatible model. When replacing the policy, update both files and the
checksums together.
