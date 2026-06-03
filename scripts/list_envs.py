import argparse

parser = argparse.ArgumentParser(description="List StackForce closed-chain USD Isaac Lab environments.")
parser.add_argument("--keyword", type=str, default=None, help="Keyword to filter environments.")
args_cli = parser.parse_args()

import gymnasium as gym
from prettytable import PrettyTable

import stackforce_mos.tasks  # noqa: F401


def main():
    table = PrettyTable(["S. No.", "Task Name", "Entry Point", "Config"])
    table.title = "Available StackForce Closed-Chain USD Environments"
    table.align["Task Name"] = "l"
    table.align["Entry Point"] = "l"
    table.align["Config"] = "l"
    index = 0
    for task_spec in gym.registry.values():
        if "ClosedUsd" in task_spec.id and (args_cli.keyword is None or args_cli.keyword in task_spec.id):
            table.add_row([index + 1, task_spec.id, task_spec.entry_point, task_spec.kwargs["env_cfg_entry_point"]])
            index += 1
    print(table)


if __name__ == "__main__":
    main()
