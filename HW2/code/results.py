import numpy as np
import argparse
import subprocess
import pandas as pd
from timeit import default_timer as timer
import random

def convertPIs(aString):
    if aString[-1] == ",":
        aString = aString[:-1]
    aString = aString.replace("pi", "3.141592")
    vecOfStrings = aString.split(",")
    return [str(round(eval(anExpression), 2)) for anExpression in vecOfStrings]  # Round to 2 decimal places

def generate_random_configuration(num_dofs, map_file):
    while True:
        config = np.round(np.random.uniform(0, 2 * np.pi, num_dofs), 2)  # Round to 2 decimal places
        config_str = ",".join(map(str, config))
        if is_valid_configuration(config_str, num_dofs, map_file):
            return config_str

def is_valid_configuration(config, num_dofs, map_file):
    command = "./config_checker.out {} {} {}".format(map_file, num_dofs, config)
    result = subprocess.run(command.split(" "), check=False).returncode
    return result == 0

def graderMain(executablePath, gradingCSV):
    maps = ["./map2.txt"]
    num_problems = 5
    num_repeats = 4
    planners = [0, 1, 2, 3]
    scores = []

    test_cases = []
    for _ in range(num_problems):
        map_file = random.choice(maps)
        num_dofs = random.randint(3, 5)
        start = generate_random_configuration(num_dofs, map_file)
        goal = generate_random_configuration(num_dofs, map_file)
        test_cases.append((map_file, num_dofs, start, goal))

    for aPlanner in planners:
        for i, (map_file, num_dofs, start, goal) in enumerate(test_cases):
            outputSolutionFile = "tmp.txt"
            startPosString = ",".join(convertPIs(start))
            goalPosString = ",".join(convertPIs(goal))
            times, costs, vertices, successes = [], [], [], 0

            for _ in range(num_repeats):
                commandPlan = "{} {} {} {} {} {} {}".format(
                    executablePath, map_file, num_dofs, startPosString, goalPosString, aPlanner, outputSolutionFile)
                commandVerify = "./verifier.out {} {} {} {} {}".format(
                    map_file, num_dofs, startPosString, goalPosString, outputSolutionFile)
                try:
                    start_time = timer()
                    subprocess.run(commandPlan.split(" "), check=True)
                    timespent = round(timer() - start_time, 2)  # Round execution time
                    times.append(timespent)

                    returncode = subprocess.run(commandVerify.split(" "), check=False).returncode
                    success = returncode == 0
                    successes += success

                    if success:
                        with open(outputSolutionFile) as f:
                            solution = [list(map(float, line.split(",")[:-1])) for line in f.readlines()[1:]]
                            solution = np.array(solution)
                            difsPos = np.abs(solution[1:] - solution[:-1])
                            cost = np.minimum(difsPos, np.abs(2*np.pi - difsPos)).sum()
                            cost = round(cost, 2)  # Round cost

                            costs.append(cost)
                            vertices.append(solution.shape[0])
                except Exception as exc:
                    print(f"Failed: {exc}")
                    times.append(5.0)
                    costs.append(-1)
                    vertices.append(-1)

            scores.append([
                aPlanner, map_file, i,
                num_dofs,  # Number of DOFs
                startPosString,  # Rounded Start Configuration
                goalPosString,  # Rounded Goal Configuration
                round(np.mean(times), 2), round(np.std(times), 2),
                round(np.mean(costs), 2), round(np.std(costs), 2),
                round(successes / num_repeats, 2)  # Round success rate
            ])

    df = pd.DataFrame(scores, columns=["planner", "mapName", "problemIndex", "numDOFs", "startConfig", "goalConfig",
                                       "meanTime", "stdTime", "meanCost", "stdCost", "successRate"])
    df.to_csv(gradingCSV, index=False)

if __name__ == "__main__":
    graderMain("./planner.out", "results.csv")
