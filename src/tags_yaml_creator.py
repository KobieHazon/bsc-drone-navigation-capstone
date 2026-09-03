import sys

command_options = ["", "flip-forward", "flip-back", "flip-right", "flip-left"]

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Error: You must specify the tags.yaml file path.")
        exit(-1)
    print("\n\nThe following is a system to create the \"tags.yaml\" file for apriltag.\nFollow the instructions "
          "below. \nYou can write \"Done\" anytime to quit.\n\n")
    input_str = ""
    res_file = open(sys.argv[1], "w")
    res_file.write("standalone_tags:\n   [\n")
    written_tags = set()
    while True:
        line = "      {"
        while True:
            try:
                input_str = raw_input("-Enter tag ID: ")
                if input_str == "Done":
                    break
                tag_id = int(input_str)
                if tag_id in written_tags:
                    raise ValueError
                else:
                    line += "id: " + input_str
                break
            except ValueError:
                print("Error: You can only enter an integer that hasn't appeared before for the tag ID.")
        if input_str == "Done":
            break
        while True:
            try:
                input_str = raw_input("-Enter tag size (in meters): ")
                if input_str == "Done":
                    break
                tmp = float(input_str)
                line += ", size: " + input_str
                break
            except ValueError:
                print("Error: You can only enter a float (the tag size is in meters).")
        if input_str == "Done":
            break
        while True:
            try:
                input_str = raw_input("-Enter tag command (options are- flip-forward/flip-back/flip-right\
                 flip-left/\"\"=no command): ")
                if input_str == "Done":
                    break
                if input_str not in command_options:
                    raise ValueError
                line += ", name: \'" + input_str + "\'}"
                break
            except ValueError:
                print("Error: You can only enter one of the following options:\n.flip-forward/flip-back/flip-right\
                 flip-left/\"\"=no command)")
        if input_str == "Done":
            break
        if len(written_tags) != 0:
            res_file.write(",\n" + line)
        else:
            res_file.write(line)
        written_tags.add(tag_id)
        print("Finished writing tag with ID=" + str(tag_id) + ".\n")

    res_file.write("\n   ]\ntag_bundles:\n   [\n   ]\n")
    res_file.close()
