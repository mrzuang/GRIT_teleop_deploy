# GRIT policy

This directory contains the GRIT deployment model and its runtime contract:

```text
checkpoints/
  policy.onnx
  policy.json
```

SHA-256 checksums:

```text
9c741a8d6ab2b7fb044e65d522fb2d6e8ae04e54eea02858f7d44fe675b37813  policy.onnx
17a273f2a2e9f5fc0e2a44d87329a181454c02ea56a33c3fc7e00e71ab96eb5c  policy.json
```

The ONNX graph must expose the multi-input deployment interface used by the
runtime:

- input `reference_context`: `[batch, 9, 70]`
- input `proprio_history`: `[batch, 990]`
- output `actions`: `[batch, 29]`

The runtime validates the ONNX input and output names directly and rejects an
incompatible model. When replacing the policy, update both files and the
checksums together.
