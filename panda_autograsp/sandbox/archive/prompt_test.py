import sys
while True:
    prompt_result = raw_input(
        "Do you want to start the planning [Y/n] ")
    # Check user input #
    # If yes download sample
    if prompt_result.lower() in ['y', 'yes']:
        print("YEEH")
    elif prompt_result.lower() in ['n', 'no']:
        print("shutdown")
        sys.exit(0)
    elif prompt_result == "":
        print("ALSO YEA")
    else:
        print(
            prompt_result + " is not a valid response please answer with Y or N to continue.")