# Deployment and inference

Use the recording configuration and demonstration contract when running a standalone ACT checkpoint. Validate the checkpoint against the environment before starting an episode. For a remote policy service, see [π0.5 deployment](openpi.md).

```bash
rwrl deploy-act --config "$SESSION_CONFIG" --checkpoint runs/act-task/checkpoints/final.pt --demo-contract demos/task.contract.json --episodes 1
```

[Deployment guide](https://github.com/JaimeParker/real-world-rl/blob/main/docs/tutorial/11-deploy.md)
