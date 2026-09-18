# Humanoid balance control

Imperial College London MSc project: simulation-based evaluation of model-based and learning-augmented humanoid balance control. This is a solution to MathWorks MATLAB and Simulink Challenge [Project 170: Simulation-Based Design of Humanoid Robots](https://github.com/mathworks/MATLAB-Simulink-Challenge-Project-Hub/tree/main/projects/Simulation-Based%20Design%20of%20Humanoid%20Robots).

## Contents

- `Codes/`: MATLAB source, Simulink models, required humanoid library/geometry and runtime parameter data.
- `Codes/FrozenControllers/`: three selected SAC policies (seed 0 episode 75; seeds 1 and 2 episode 25).
- `Report Content/Figures/`: images included in the thesis chapters.
- `Report Content/Data/`: only saved inputs consumed by the report figure generators, with unused MAT variables omitted. These are figure-reproduction inputs, not a complete experiment archive.

No technical records, planning documents, bulk training checkpoints or saved-results tree are included. New simulations create their own `Codes/Results/` outputs locally.

Within `Codes/`, files are grouped into `setup`, `models`, `controllers`, `training`, `experiments`, `analysis`, `plotting`, `model_construction` and `assets`. Selected agents retain their `FrozenControllers` paths. The supplied Simulink files are the completed models; development-only scripts that programmatically added their blocks are intentionally excluded. `model_construction` retains only the runtime RBT builder and its scientific validation.

## Requirements and setup

The tested local environment is MATLAB R2025b. For the complete set of retained studies, install Simulink, Simscape, Simscape Multibody, Robotics System Toolbox, Reinforcement Learning Toolbox, Deep Learning Toolbox and Simscape Electrical (Motor & Drive components). [Reinforcement Learning Toolbox requires Deep Learning Toolbox](https://www.mathworks.com/support/requirements/reinforcement-learning-toolbox.html).

A static audit of the retained MATLAB source and model files identified 74 dependency files, with no detected dependencies outside this candidate and the MATLAB installation. It directly identified MATLAB, Simulink, Robotics System Toolbox and Reinforcement Learning Toolbox. This is not an exhaustive runtime guarantee: model blocks, serialized agents and dynamically assembled file paths can require dependencies beyond the static listing. Setup, model compilation and figure-reproduction checks are recorded below.

Open MATLAB in the repository root and run:

```matlab
setup_project
```

This adds the code/model/asset folders to MATLAB's path and changes the working directory to `Codes/`, where new experiment outputs are written. Run it once per fresh MATLAB session. Stable root helpers keep file lookup independent of each script's subfolder. Model callbacks currently emit warnings for obsolete `ImportedURDFSupport` paths; do not download unrelated assets solely to silence them.

Install Git LFS before cloning this repository:

```text
git lfs install
git clone <repository-url>
cd <repository-folder>
git lfs pull
```

The three trained agents must be real MAT files, not small LFS pointer text files. A ZIP/source-only download may not contain the agent binaries; cloning with Git LFS is the supported route.

The rigidBodyTree has 22 coordinates (six floating-base, four shoulder, twelve leg). Plant x is lateral and y is sagittal. Inherited joint names and validated left-side signs must not be interpreted using anatomical naming alone.

## One-command demonstration

From MATLAB, with the repository root as the current folder, run:

```matlab
results = run_project_demo();
```

The demonstration performs four matched 12-second simulations: nominal and a
+150 N lateral push for Controller A and the frozen deterministic RL75 policy.
It initializes the project, verifies the packaged dependencies, loads the
pretrained policy, applies the frozen disturbance protocol, classifies both
responses and plots force-induced lateral torso displacement. Runtime depends
on the host and Simscape solver performance and may be several minutes.

Expected classification:

| Controller | +150 N lateral push |
| --- | --- |
| Controller A | Fell |
| Controller A + RL75 | Recovered |

The command prints `Demo verification passed` when these outcomes are
reproduced. The warnings about obsolete `ImportedURDFSupport` paths are known
model-callback warnings and do not prevent simulation.

## Inspect/setup the models

```matlab
humanoid_walker_parameters
inverseDynamics
residual_rl_parameters
[agentObj, controller] = load_frozen_residual_rl_agent();
open_system("HumanoidModel_ResidualRL")
```

Use the experiment functions below for explicit frozen controller settings, rather than assuming manual workspace setup alone applies a complete comparison configuration.

The release A/D parameter snapshot contains the exact previously selected commands/correction values. Release policy loaders check agent identity and `release_runtime_checksums.csv`, not the presence of private historical outputs. Historical hashes retained in source describe the original experiments; the release checks describe the later deployment files. These are different provenance layers, not a claim that every historical file is unchanged.

## Run comparisons

```matlab
results = run_formal_robustness_benchmark();
combined = run_final_rl75_frozen_protocol_benchmark("", results.resultDirectory);
```

This reruns the A/D benchmark, then runs RL75 and consolidates the comparison using the newly generated A/D directory. Other functions that consume prior results likewise need new input paths instead of hard-coded historical defaults.

Other entry points:

- `run_frozen_rl75_authority_ablation`: same policy with changed physical torque authority.
- `benchmark_all_motor_drive_nominal_controllers`: A/D/RL75 nominal actuator comparison.
- `benchmark_all_motor_drive_disturbance`: matched actuator disturbance comparison.
- `run_model_uncertainty_study`: model uncertainty comparison.
- `train_residual_rl_sac`, `train_residual_rl_authority_ablation`: stochastic training.
- `evaluate_residual_rl_checkpoints`: checkpoint screening/evaluation, with the training output directory supplied explicitly.

Some analysis wrappers retain historical default input paths. Supply newly generated results when using them. They are not all standalone commands. Detailed input contracts are in each function's arguments/help. No claim is made that all wrapper workflows have passed a fresh full-length rerun.

Final common disturbance comparisons use the frozen 12-second protocol. Separate nominal actuator follow-ups used longer horizons. Retraining does not guarantee identical learned weights or recovery performance.

## Results

Under the frozen finite-horizon benchmark, Controller A demonstrated
bidirectional lateral recovery at 100 N. The selected residual policy RL75
extended the largest tested bidirectionally recovered lateral disturbance to
185 N. Deploying the same policy with residual torque limits of 5, 10 and
15 N m produced tested recovery levels of 100, 150 and 185 N, respectively,
showing that the improvement depended on available residual authority.

![Controller A and RL75 lateral response](<Report Content/Figures/Chapter 4/controller_A_RL75_lateral_response.png>)

![Residual-authority comparison](<Report Content/Figures/Chapter 4/residual_authority_comparison.png>)

The actuator-nonideality study replaced ideal torque delivery on the active
controlled joints with system-level Motor & Drive components. Its selected
nominal and disturbed comparisons are provided as representative robustness
tests rather than hardware or sim-to-real validation.

![Actuator-nonideality disturbance outcomes](<Report Content/Figures/Chapter 5/nonideal_disturbance_outcome_map.png>)

## Regenerate report figures

After `setup_project`, with report data present:

```matlab
generate_chapter3_nominal_figures
generate_chapter3_robustness_figures
generate_chapter4_residual_figures
generate_chapter5_actuator_figures
```

Generators read `Report Content/Data/` and write `Report Content/Figures/`. They do not simulate or retune controllers. The retained paths reflect study/run identities for traceability, not a general saved-results archive.

## Verification and release status

Release verification passed complete A/D-definition equality with the original, 22-coordinate setup, all three selected-policy loaders and all four Chapter 3–5 figure generators using relocated report inputs. MAT exports were checked for exact value equality. After folder organization, fresh setup, all three policy loaders, all three main model diagram updates and all four thesis figure generators passed. The actuator-nonideality model subsequently received a presentation-only cleanup that removed disconnected line fragments and unused Constant/converter pairs; its functional actuator connections were compared with the original and its diagram update passed. The models also passed three 0.25-second zero-force startup simulations in the broader staging copy before folder organization. The root `run_project_demo` entry point passed its complete four-simulation expected-outcome check. Full benchmark-grid and poster-generator reruns have not been performed after packaging.

Model callbacks still emit warnings for obsolete `ImportedURDFSupport` paths. Startup success does not verify full recovery performance. Tested recovery levels are not proven continuous disturbance limits, and representative nonideal-actuation tests are not hardware/sim-to-real validation.

## Large files, licence and attribution

Only the three selected trained agents use Git LFS, configured by `.gitattributes`. Models, runtime parameters and compact report inputs remain in ordinary Git. Their locations are unchanged for MATLAB. LFS account availability/storage/download allowances must be checked before publishing; no upload has been performed during local preparation.

Original project contributions are released under the [BSD 2-Clause License](LICENSE). The humanoid model, library and geometry inherit MathWorks example resources; see [Third-party notices](THIRD_PARTY_NOTICES.md).
