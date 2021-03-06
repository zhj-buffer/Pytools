import glob
import os
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=str, required=True, help="Name of the dataset such as manual, voc and mpii")
    parser.add_argument("--data_dir", type=str, default="outputs", help="Directory containing training data")
    args = parser.parse_args()

    dl = ['labels_voc2007','labels_voc2012']
    for x in dl:
        label_dir = os.path.join(args.data_dir, args.dataset, x)
        output_file = os.path.join(args.data_dir, "{}.txt".format(args.dataset))
        label_files = glob.glob("{}/*.txt".format(label_dir))

        print output_file

        with open(output_file, 'aw') as f:
            for label_file in label_files:
                image_file = label_file.replace("/labels", "/images")
                image_file = image_file.replace(".txt", ".jpg")
                #print image_file, label_file
                if not os.path.isfile(image_file):
                    print image_file
                    #raise Exception("Somethign wrong!!!")
                    continue
                f.write(os.path.join(os.getcwd(), "{}\n".format(image_file)))


if __name__ == "__main__":
    main()
