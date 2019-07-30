"""A small test script I made to test the argparser. Based on https://docs.python.org/2/howto/argparse.html"""

## Standard library imports ##
import argparse

#################################################
## Parse arguments ##############################
#################################################

### Approach 1 ###
# parser = argparse.ArgumentParser()
# parser.add_argument("square",  type=int,
#                     help="display a square of a given number")
# parser.add_argument("-v", "--verbosity", type=int, choices=[0, 1, 2],
#                     help="increase output verbosity")
# args = parser.parse_args()
# answer = args.square**2
# if args.verbosity == 2:
#     print("the square of {} equals {}".format(args.square, answer))
# elif args.verbosity == 1:
#     print("{}^2 == {}".format(args.square, answer))
# else:
#     print(answer)

### Approach 2 CPython like ###
# parser = argparse.ArgumentParser()
# parser.add_argument("square", type=int,
#                     help="display the square of a given number")
# parser.add_argument("-v", "--verbosity", action="count",
#                     default=0, help="increase output verbosity")
# args = parser.parse_args()
# answer = args.square**2
# if args.verbosity >= 2:
#     print("the square of {} equals {}".format(args.square, answer))
# elif args.verbosity >= 1:
#     print("{}^2 == {}".format(args.square, answer))
# else:
#     print(answer)

### Advanced 1 ###
# parser = argparse.ArgumentParser()
# parser.add_argument("x", type=int, help="the base")
# parser.add_argument("y", type=int, help="the exponent")
# parser.add_argument("-v", "--verbosity", action="count", default=0)
# args = parser.parse_args()
# answer = args.x**args.y
# if args.verbosity >= 2:
#     print ("Running '{}'".format(__file__))
# if args.verbosity >= 1:
#     print ("{}^{} ==".format(args.x, args.y), end=" ", flush=True)
# print(answer)

### Conflicting options ###
parser = argparse.ArgumentParser(description="Calculate X to the power of Y")
group = parser.add_mutually_exclusive_group()
group.add_argument("-v", "--verbose", action="store_true")
group.add_argument("-q", "--quiet", action="store_true")
parser.add_argument("x", type=int, help="the base")
parser.add_argument("y", type=int, help="the exponent")
args = parser.parse_args()
answer = args.x**args.y
if args.quiet:
    print(answer)
elif args.verbose:
    print("{} to the power {} equals {}".format(args.x, args.y, answer))
else:
    print("{}^{} == {}".format(args.x, args.y, answer))
