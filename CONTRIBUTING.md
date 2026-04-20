# Contributing Guide — G.O.K.U Team

> Office Fetch Robot · Mobile Robotics MSc · University of Bonn

------

## Branch strategy

`main` must always compile and run. All development happens on feature branches; merges go through Pull Requests with at least **one reviewer who is not the author**.

```
main
 ├── feature/aruco-detection
 ├── feature/occupancy-grid
 ├── feature/path-planner
 ├── feature/unet-training
 ├── feature/gazebo-setup
 └── docs/devlog-weekN
```

Branch naming:

| Prefix      | Use                               |
| ----------- | --------------------------------- |
| `feature/`  | New functionality                 |
| `fix/`      | Bug fixes                         |
| `exp/`      | Experiments and ablations         |
| `refactor/` | Code cleanup, no behaviour change |
| `docs/`     | Documentation and devlogs         |

### Daily workflow — step by step

**1. Before you start working, sync with main**

```Bash
git checkout main
git pull origin main
```

**2. Create your feature branch**

```Bash
git checkout -b feature/your-feature-name
# Example: git checkout -b feature/aruco-detection
```

**3. Work, then stage and commit**

```Bash
git add .                          # stage all changes
# or: git add src/aruco/ docs/     # stage specific files only
git commit -m "feat(aruco): implement marker detection using OpenCV ArUco"
```

**4. Push your branch to GitHub**

```Bash
# First push (sets upstream):
git push -u origin feature/your-feature-name

# Subsequent pushes:
git push
```

**5. Keep your branch up to date with main (avoid large merge conflicts)**

```Bash
git checkout main
git pull origin main
git checkout feature/your-feature-name
git merge main

# Resolve any conflicts, then:
git add .
git commit -m "chore: merge latest main into feature branch"
git push
```

**6. When your feature is ready — open a Pull Request on GitHub**

Go to the repository on GitHub → **"Compare & pull request"** → fill in the PR template → assign a reviewer.

**7. After your PR is merged — clean up**

```Bash
git checkout main
git pull origin main
git branch -d feature/your-feature-name    # delete local branch
```

------

## Commit convention

Format: `<type>(<scope>): <short description>`

| Type       | Use                                                |
| ---------- | -------------------------------------------------- |
| `feat`     | New feature                                        |
| `fix`      | Bug fix                                            |
| `docs`     | Documentation / devlog update                      |
| `exp`      | Experiment run (include key metric in description) |
| `refactor` | Refactoring                                        |
| `chore`    | Config, dependencies, maintenance                  |

Examples:

```
feat(aruco): implement marker detection using OpenCV ArUco
fix(mapping): resolve memory leak in occupancy grid update
exp(unet): lr=0.001 batch=16 dice_loss → 78.3% mIoU
docs(devlog): add week 3 progress for path planning
```

------

## Pull Request checklist

Before opening a PR:

- [ ] Code runs without errors
- [ ] Devlog updated (`docs/devlog/YYYY-MM-DD_title.md`)
- [ ] PR description explains what changed and how to test it
- [ ] Closes the relevant GitHub Issue (`Closes #N`)
- [ ] Reviewed by at least one teammate

------

## DevLog

DevLogs are our most important documentation artifact. They feed directly into presentations, the final report, and serve as a record of individual contribution.

**Location:** `docs/devlog/YYYY-MM-DD_short-title.md`

### Creating a devlog entry

```bash
# Make sure you're on your feature branch 
touch docs/devlog/2026-04-20_aruco-first-test.md 
# Write your entry, then commit: 
git add docs/devlog/2026-04-20_aruco-first-test.md 
git commit -m "docs(devlog): add aruco first test notes" 
git push
```

**Template:**

```markdown
# YYYY-MM-DD · Title

**Author:**
**Module:** aruco_localization | occupancy_mapping | path_planning | vision | integration
**Type:** feature | bugfix | experiment | decision

## Objective
What you aimed to do today.

## Process
What you tried, commands run, design decisions made.

## Problems encountered

- **Problem 1**
  - Cause:
  - Solution:

## Results
Success / partial / failure. Include screenshots, metrics, or log snippets where applicable.

## Key learnings
One or two sentences on what you now understand better.

## Next steps
What comes next, and any open questions.
```

Write a devlog entry for every significant session. Short entries are fine — the habit matters more than the length.

------

## Milestones

| Milestone          | Calendar week | Target date      |
| ------------------ | ------------- | ---------------- |
| Presentation 1     | CW19          | 3 May 2026       |
| Presentation 2     | CW27          | early July 2026  |
| Final presentation | CW36          | 1 September 2026 |

Use GitHub Milestones to track Issues against these dates.

------

## Weekly rhythm

**Monday — 15 min stand-up**

1. What did you do last week?
2. What will you do this week?
3. Any blockers?

**Midweek** — independent work; ask questions in the group chat anytime.

**Thursday — 30–60 min meeting**

- Demo progress
- Code review
- Update project plan
- Write or review devlog summaries

**Before every supervisor meeting:**

- Prepare a written agenda (`docs/meeting-notes/YYYY-MM-DD_agenda.md`)
- Bring specific, concrete questions
- Record action items and owners immediately after

------

## Experiment tracking

For U-Net and any other training runs, store results under `experiments/`:

```
experiments/unet_training/
├── exp_001_baseline/
│   ├── config.yaml      ← required for reproducibility
│   ├── train_log.txt
│   ├── metrics.json
│   └── notes.md
├── exp_002_augmentation/
└── exp_003_dice_loss/
```

Each experiment needs a `config.yaml` with all hyperparameters. Commit the config and metrics even if the result is negative — negative results are data.

```bash
mkdir -p experiments/unet_training/exp_001_baseline 
# Add your config and results, then: 
git add experiments/unet_training/exp_001_baseline/ 
git commit -m "exp(unet): lr=0.001 batch=16 dice_loss → 78.3% mIoU" 
git push
```

------

## Reviewer availability
If the assigned reviewer is unavailable for more than 48 hours, 
any other team member may approve the PR to avoid blocking progress.

------

> Supervisors evaluate not only results, but also methodology, decision-making, and collaboration. Treat the entire process as a well-managed engineering project.