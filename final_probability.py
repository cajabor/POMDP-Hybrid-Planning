import pandas as pd

def belief(input_file):
    # Read the CSV file
    df = pd.read_csv(input_file)
    
    # Initialize lists to hold the new table values
    distance_changed = []
    has_bumped = []
    door_passed = []
    chunk_size = 3
    
    # Iterate over the dataframe in chunks of 10
    for i in range(0, len(df), chunk_size):
        chunk = df.iloc[i:i + chunk_size]
        
        # Calculate the distance change (absolute difference between first and last Current_Distance)
        if len(chunk) > 1:
            change = abs(chunk['Current_Distance'].iloc[0] - chunk['Current_Distance'].iloc[-1])
        else:
            change = 0  # If there's only one entry in the chunk
        
        # Determine if has_bumped is true in any of the next 10 entries
        bump = chunk['Bumper_Triggered'].any()

        door_value = chunk['Door_Passed'].iloc[0] if not chunk['Door_Passed'].isnull().all() else False

        
        # Append results to the lists
        distance_changed.append(True) if change > 7 else distance_changed.append(False)
        has_bumped.append(bump)
        door_passed.append(door_value)


    # Create a new DataFrame for the results
    result_df = pd.DataFrame({
        'Distance_Changed': distance_changed,
        'Has_Bumped': has_bumped,
        'Door_Passed': door_passed
    })
    
    # Initialize a dictionary to compute probabilities
    probability_counts = {}     
    
    # Calculate the probabilities
    for bump in [True, False]:
        for change in [True, False]:
            subset = result_df[(result_df['Has_Bumped'] == bump) & (result_df['Distance_Changed'] == change)]
            if not subset.empty:
                prob_door_passed = subset['Door_Passed'].mean()
                key = f"{(change)} {(bump)}"
                probability_counts[key] = prob_door_passed

    # Create a probability DataFrame
    probability_table = pd.DataFrame({
        'Dist_Change Bump': probability_counts.keys(),
        'P(Door)': [probability_counts[key] for key in probability_counts.keys()]
    })
    
    # Print the result table
    print("Result DataFrame:")
    print(result_df)
    print("\nCPT Table:")
    print(probability_table)

# Example usage
belief('final_cpt_data.csv')
