def print_number_list(num_list, decimal=5):
    string = ""
    for num in num_list:
        string += str(round(num, decimal)) + ", "
    string = string[:-2]
    print(string)
    return string
