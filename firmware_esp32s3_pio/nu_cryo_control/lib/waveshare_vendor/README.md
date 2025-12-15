# waveshare_vendor

Copy Waveshare-provided source files here **as-is** (no edits).

Keep vendor code isolated from application logic. Wrap any required calls in
`waveshare_hal` so your core firmware never directly includes vendor headers.
