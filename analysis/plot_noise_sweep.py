from .plot_sweep import plot_sweep


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("campaign")
    parser.add_argument("--output-dir")
    args = parser.parse_args(argv)
    print(plot_sweep(args.campaign, "noise_std_mm", args.output_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
