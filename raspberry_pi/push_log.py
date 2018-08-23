import dropbox
import sys

class TransferData:
    def __init__(self, access_token):
        self.access_token = access_token

    def upload_file(self, file_from, file_to):
        """upload a file to Dropbox using API v2
        """
        dbx = dropbox.Dropbox(self.access_token)

        with open(file_from, 'rb') as f:
            dbx.files_upload(f.read(), file_to)

def main():
    line = open('dropboxKey.txt').readline()
    access_token = line.rstrip()
    transferData = TransferData(access_token)
    if(len(sys.argv) != 2):
      print("give me a log file as argument please!")
      exit()

    print("pushing {}...".format(sys.argv[1]))
    file_from = sys.argv[1]
    file_to = "/beakerLogs/" + sys.argv[1]

    # API v2
    transferData.upload_file(file_from, file_to)
    print("Done!")

if __name__ == '__main__':
    main()
