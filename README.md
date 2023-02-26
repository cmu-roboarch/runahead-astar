# Runahead A\*

This repository contains an implementation of the following paper:

```
@inproceedings{bakhshalipour:icaps:2023:rastar,
    title = {{Runahead A*: Speculative Parallelism for A* with Slow Expansions}},
    author = {Bakhshalipour, Mohammad and Qadri, Mohamad and Guri, Dominic and Ehsani, Seyed Borna and Likhachev, Maxim and Gibbons, Phillip},
    booktitle = {International Conference on Automated Planning and Scheduling (ICAPS)},
    year = {2023}
}
```

## 0. Replicate The Results
Running `python3 replicate.py` must produce a graph showing RA\*'s speedup, similar to Fig. 4 of the manuscript. `results/speedup_xeon_gold.png` shows the results on a system with an Intel Xeon Gold 5218R processor and a Debian 10 OS. And, `results/speedup_xeon_e5.png` shows the results on another system with an Intel Xeon E5-2670 processor and an Ubuntu 18.04 OS. Running the script on other processors with >= 16 cores must produce similar results.

Running the script on our machines takes ~20 minutes. To reduce the execution time, please reduce the number of tests by changing `NUM_TESTS = 10` in `replicate.py`. Please ensure that no other applications are running on the system while running the script.

To reproduce the other speedup graphs, please change the corresponding variables in the `replicate.py` script. For example, to reproduce the **Planning Resolution** experiment, please change `MAP_FILES` to `["input-obs/*"]`. Or, to reproduce the **Weighted A** graph, please change `WEIGHTS` to `[1, 2, 4]`.

## 1. Source Code
`xyplan.cpp` is the source code of the baseline A\* and RA\*. The other \*.cpp/.h files are auxiliary. `input-obs/` includes the environment maps with different resolutions (i.e., different _number_ of obstacles). And, `Makefile` includes the metaprogramming of the compilation workflow including the compile flags we use in our experiments.

## 2. Build
To evaluate RA\* beyond the `replicate.py` script, please build the repository. Issuing `make performance` will build the repository for maximizing performance, optimizing out unnecessary codes. Issuing `make` (without any argument) will build the entire source code, including the verification codes (e.g., `assert`, etc.).

## 3. Run
Run `./xyplan.out -h` to get a list of available arguments. Essentially, the input map, number of threads, weighted A\* weight (epsilon), etc. can be configured from the command line.
