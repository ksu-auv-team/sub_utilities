import os
import argparse
import pandas as pd
from sklearn.model_selection import train_test_split

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Separates a CSV file into training and validation sets',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        'input_csv',
        metavar='input_csv',
        type=str,
        help='Path to the input CSV file')
    parser.add_argument(
        '-f',
        metavar='train_frac',
        type=float,
        default=.75,
        help=
        'fraction of the dataset that will be separated for training (default .75)'
    )
    parser.add_argument(
        '-s',
        metavar='stratify',
        type=bool,
        default=True,
        help='Stratify by class instead of whole dataset (default True)')

    args = parser.parse_args()

    if args.f < 0 or args.f > 1:
        raise ValueError('train_frac must be between 0 and 1')
    
    df = pd.read_csv(args.input_csv)

    # get 'class' column for stratification
    strat = df['class'] if args.s else None

    train_df, validation_df = train_test_split(
        df, test_size=None, train_size=args.f, stratify=strat)

    # output files have the same name of the input file, with some extra stuff appended
    new_csv_name = os.path.splitext(args.input_csv)[0]
    train_csv_path = new_csv_name + '_train.csv'
    eval_csv_path = new_csv_name + '_eval.csv'

    train_df.to_csv(train_csv_path, index=False)
    validation_df.to_csv(eval_csv_path, index=False)
