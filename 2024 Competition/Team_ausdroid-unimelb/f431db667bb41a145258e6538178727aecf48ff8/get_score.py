import sys
import json



if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Please check the number of files")
    else:
        total_complete = 0
        total_reveal = 0

        try:
            for i in range(1, len(sys.argv)):
                with open(sys.argv[i], 'r') as f:
                    result = json.load(f)
                
                current_complete = result["numTaskFinished"]
                current_reveal = result["tasks"][-1][0]
                total_complete += current_complete
                total_reveal += current_reveal

                print(f"Score for {sys.argv[i]}: {str(round(current_complete/current_reveal, 2))}")

        except:
            print("Please check file name")

        
        print(f"Total Score: {str(round(total_complete/total_reveal, 2))}")