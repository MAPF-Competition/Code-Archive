import argparse
import datetime
import os 
import tempfile
import subprocess

def open_editor(initial_message=""):
    # Get the editor from the environment variable; default to 'vim' if not set
    editor = os.environ.get("EDITOR", "vim")
    
    # Create a temporary file
    with tempfile.NamedTemporaryFile(suffix=".tmp", mode="w+", delete=False) as tmp_file:
        tmp_filename = tmp_file.name
        # Write any initial text if provided
        if initial_message:
            tmp_file.write(initial_message)
            tmp_file.flush()
    
    # Open the editor with the temporary file
    subprocess.call([editor, tmp_filename])
    
    # Read the content after the editor is closed
    with open(tmp_filename, "r") as tmp_file:
        content = tmp_file.read()
    
    # Remove the temporary file
    os.unlink(tmp_filename)
    
    return content

def main():
    parser = argparse.ArgumentParser(
                    prog='Notter',
                    description='adds updates, thoughts, to-dos to the project',
                    epilog='')
    parser.add_argument('-n', type=str)      # option that takes a value

    args = parser.parse_args()

    msg = args.n.strip() if args.n != None else open_editor() 
    limit = 100
    if len(msg.split(' ')) > limit: 
        print(f"size of the message can't exceed {limit} words")
        exit()
    time = datetime.datetime.now()
    with open("notes.md", "a") as notes_file:
        s = ""
        s += f"[{time}] {msg}\n"
        notes_file.write(s)
    print(args.n)

if __name__ == "__main__":
    main()