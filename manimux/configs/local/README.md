# Station templates

Use the [station guide](../../../.agents/skills/manimux-station-setup/references/station.md) for device bindings,
service addresses, checkpoint paths and configuration inspection.

- `yam_example.yaml`: YAM with the configured camera services.
- `tianji_taccap_example.yaml`: Tianji controller and TacCap components.
- `piper_example.yaml`: standard PiPER CAN and measured width calibration.
- `arx_x5_example.yaml`: X5 (2023) CAN, separate encoder/command calibration and
  explicit SDK polling selection. See [SDK adapter limits](../../../.agents/skills/manimux-station-setup/references/can-arms.md).
- Copy the matching template to `station.yaml`, the default private station file.

AsyncSim uses `asyncsim.example.yaml`, `asyncsim_xr1.example.yaml`,
`asyncsim_openwam.example.yaml` or `asyncsim_vpp2.example.yaml` for service and
artifact paths. These templates do not open hardware. XR1's `paths.model_dir`
selects its Hugging Face 30-step X5 bundle. See the
[VPP2 deployment reference](../../../.agents/skills/manimux-inference/references/deployment/vpp2-arx-x5.md).

Template names use underscores; retain the `.yaml` extension. Additional private
stations can live under the ignored root `.local/` directory, for example
`.local/station_lab.yaml`, selected with `--local` on each relevant process.
