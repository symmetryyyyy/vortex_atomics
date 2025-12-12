import os
import subprocess
import argparse
from datetime import datetime

# Parse command line arguments
parser = argparse.ArgumentParser(description='Run Vortex blackbox tests')

parser.add_argument('--cores', type=int, nargs='+', help='Core counts (e.g., --cores 4 8)')
parser.add_argument('--warps', type=int, nargs='+', help='Warp counts')
parser.add_argument('--threads', type=int, nargs='+', help='Thread counts')
parser.add_argument('--sim', type=str, nargs='+', help='Simulators (e.g., --sim rtlsim simx)')
parser.add_argument('--app', type=str, nargs='+', help='Applications')
parser.add_argument('--size', type=int, nargs='+', help='Sizes')
# parser.add_argument('--debug', type=int, help='debug level')
args = parser.parse_args()

# Default configurations (can be overridden by command line)
core_list = args.cores if args.cores else [4]
warp_list = args.warps if args.warps else [4, 8]
thread_list = args.threads if args.threads else [4, 8]
sim_list = args.sim if args.sim else ["simx", "rtlsim"]
app_list = args.app if args.app else ["sgemm2","sgemm_async"]
size_list = args.size if args.size else [16,32,64]
# debug_level = args.debug if args.debug else 3

# Create log directory with timestamp
timestamp = datetime.now().strftime("%H%M%S")
log_dir = f"test_logs_{''.join(sim_list)}_{''.join(app_list)}_{''.join(map(str, size_list))}_{timestamp}"
os.makedirs(log_dir, exist_ok=True)

# Summary file
summary_file = os.path.join(log_dir, "summary.txt")
summary = open(summary_file, "w")
summary.write(f"Test Summary - {timestamp}\n")
summary.write("=" * 80 + "\n\n")

# Statistics
passed = 0
failed_numerical = 0
failed_crash = 0
failed_other = 0

def classify_failure(log_file):
    """Classify the type of failure by examining the log"""
    try:
        with open(log_file, 'r') as f:
            content = f.read()
            # Check for crash/abort (architecture-level bugs)
        if 'Aborted' in content or 'core dumped' in content:
            return 'CRASH', 'Aborted (core dumped) - likely RTL bug'
        if 'Assertion failed' in content:
            return 'CRASH', 'Assertion failed - RTL timeout or logic error'
        if 'Segmentation fault' in content:
            return 'CRASH', 'Segmentation fault'
        # if '%Error:' in content and 'VX_schedule' in content:
        #     return 'CRASH', 'Scheduler timeout - warps deadlocked'
        
        # Check for numerical errors (less critical)
        if 'FAILED' in content and 'errors' in content.lower():
            return 'NUMERICAL', 'Numerical mismatch in results'
        
        # Unknown failure
        return 'OTHER', 'Unknown failure type'
    except:
        return 'OTHER', 'Could not read log file'

for core in core_list:
    for warp in warp_list:
        for thread in thread_list:
            for sim in sim_list:
                for app in app_list:
                    for size in size_list:
                    # Generate test name and log file
                        test_name = f"c{core}_w{warp}_t{thread}_{sim}_{size}_{app}"
                        log_file = os.path.join(log_dir, f"{test_name}.log")

                        # cmd = f"./ci/blackbox.sh --cores={core} --warps={warp} --threads={thread} --driver={sim} --app={app} --debug={debug_level}"
                        cmd = f"./ci/blackbox.sh --cores={core} --warps={warp} --threads={thread} --driver={sim} --app={app} --args=-n{size}"

                        print(f"\n{'='*80}")
                        print(f"Running: {test_name}")
                        print(f"Command: {cmd}")
                        print(f"Log: {log_file}")

                        # Run command and capture output
                        with open(log_file, "w") as f:
                            result = subprocess.run(
                                cmd,
                                shell=True,
                                stdout=f, 
                                stderr=subprocess.STDOUT, 
                                timeout=2400  # 40 minute timeout
                            )

                        # Check result and classify failure
                        if result.returncode == 0:
                            status = "PASSED"
                            reason = "Success"
                            passed += 1
                            print(f"✓ {status}")
                        else:
                            failure_type, reason = classify_failure(log_file)

                            if failure_type == 'CRASH':
                                status = "FAILED [CRASH]"
                                failed_crash += 1
                                print(f"✗ {status} - {reason}")
                            elif failure_type == 'NUMERICAL':
                                status = "FAILED [NUMERICAL]"
                                failed_numerical += 1
                                print(f"⚠ {status} - {reason}")
                            else:
                                status = "FAILED [OTHER]"
                                failed_other += 1
                                print(f"✗ {status} - {reason}")

                        # Write to summary
                        summary.write(f"[{status}] {test_name}\n")
                        summary.write(f"  Command: {cmd}\n")
                        summary.write(f"  Log: {log_file}\n")
                        summary.write(f"  Exit code: {result.returncode}\n")
                        summary.write(f"  Reason: {reason}\n\n")

# Final summary
total = passed + failed_numerical + failed_crash + failed_other
print(f"\n{'='*80}")
print(f"Test Results Summary:")
print(f"  Total:              {total}")
print(f"  ✓ Passed:           {passed}")
print(f"  ✗ Failed (Crash):   {failed_crash}  <-- Priority: Architecture bugs")
print(f"  ⚠ Failed (Numerical): {failed_numerical}  <-- Priority: Result accuracy")
print(f"  ✗ Failed (Other):   {failed_other}")
print(f"  Log directory: {log_dir}")
print(f"  Summary file: {summary_file}")

summary.write("=" * 80 + "\n")
summary.write(f"Total:              {total}\n")
summary.write(f"Passed:             {passed}\n")
summary.write(f"Failed (Crash):     {failed_crash}\n")
summary.write(f"Failed (Numerical): {failed_numerical}\n")
summary.write(f"Failed (Other):     {failed_other}\n")
summary.close()

# Exit with non-zero if any test failed
exit(0 if (failed_crash + failed_numerical + failed_other) == 0 else 1)