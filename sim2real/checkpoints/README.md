# GRIT policy

This directory contains the GRIT deployment model and its runtime contract:

```text
checkpoints/
  policy.onnx
  policy.json
```

SHA-256 checksums:

```text
39d573e970b7088275ed68ec8a9597fa9c402d98d5ad211991db7d8d0f5a82b9  policy.onnx
8a1e5795c898075e4e9263fdff3e85c34747fdb10aa84cacd0af540be7b3dbd4  policy.json
```

The ONNX graph must expose the multi-input deployment interface used by the
runtime:

- input `reference_context`: `[batch, 9, 70]`
- input `proprio_history`: `[batch, 990]`
- output `actions`: `[batch, 29]`

The runtime validates the ONNX input and output names directly and rejects an
incompatible model. When replacing the policy, update both files and the
checksums together.
