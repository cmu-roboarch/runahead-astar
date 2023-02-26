#! /usr/bin/env python3

import matplotlib.pyplot as plt
import glob
import shlex, shutil
import subprocess as sp
import os, sys
import re

BINARY = "xyplan.out"

MAP_FILES = ["input-obs/Boston_1024_1024_d001.obs"]
# MAP_FILES = ["input-obs/*"]
WEIGHTS = [1]
# WEIGHTS = [1, 2, 4]

NUM_TESTS = 10

def extractExecTime(log):
    for l in log.splitlines():
        m = re.search("execTime:.(\d*\.*\d*)", l)
        if not m: continue
        return float(m.group(1))

    assert False, "Could not extract the execution time"


inputMapFiles = set()
for mapFile in MAP_FILES:
    for f in glob.glob(mapFile):
        inputMapFiles.add(f)


print("\nCleaning the object files if any...")
proc = sp.run(shlex.split("make clean"))
assert proc.returncode == 0

print("\nBuilding the code")
proc = sp.run(shlex.split("make performance"))
assert proc.returncode == 0

assert os.path.exists(BINARY)
assert len(inputMapFiles) > 0

numThreads = [2**i for i in range(1, 5)]

for inputFile in inputMapFiles:
    for w in WEIGHTS:
        execTimeBaseline = []
        execTimeRAstar = []

        experimentSignature = f"map:{os.path.basename(inputFile)} numTests:{NUM_TESTS} weight:{w}"

        for t in numThreads:
            print(f"\nRunning {experimentSignature} with {t} threads without speculation")
            cmd = f"./{BINARY} --map={inputFile} --num-tests={NUM_TESTS} --weight={w} --threads={t}"
            proc = sp.run(shlex.split(cmd), stdout=sp.PIPE)
            assert proc.returncode == 0
            log = proc.stdout.decode().strip()
            execTimeBaseline.append(extractExecTime(log))

            print(f"\nRunning {experimentSignature} with {t} threads with speculation")
            cmd += " --speculation"
            proc = sp.run(shlex.split(cmd), stdout=sp.PIPE)
            assert proc.returncode == 0
            log = proc.stdout.decode().strip()
            execTime = extractExecTime(log)
            execTimeRAstar.append(extractExecTime(log))

        normPerf = [j/i for i,j in zip(execTimeRAstar, execTimeBaseline)]
        print(f"\nRA*'s normalized performance with {experimentSignature}:", normPerf)

        plt.title(experimentSignature)
        plt.xticks(numThreads, [str(t) for t in numThreads])
        plt.plot(numThreads, normPerf)
        plt.draw()
        plt.pause(.0001)
        plt.savefig("rastar_perf_{}.png".format(experimentSignature.replace(' ', '_').replace(':', '')))
        plt.clf()

plt.pause(1000)
