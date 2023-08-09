import argparse

from algorithms.single_agent import dqn

parser = argparse.ArgumentParser()

parser.add_argument("--algo",
                    type=str,
                    default="dqn",
                    choices=["dqn", "ppo", "iql"],
                    help="The algorithm to be used")
parser.add_argument("--mode",
                    type=str,
                    default="eval",
                    choices=["train", "eval"],
                    help="The mode of inference")

parser.add_argument("--agent",
                    type=str,
                    default="single",
                    choices=["single", "multi"],
                    help="Type of agents used"
                    )


def main():
    args = parser.parse_args()

    if args.agent == "single":
        if args.algo == "dqn":
            dqn.train() if args.mode == "train" else dqn.evaluate()
        elif args.algo == "ppo":
            dqn.train() if args.mode == "train" else dqn.evaluate()
    else:
        pass


if __name__ == '__main__':
    main()
