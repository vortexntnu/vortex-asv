import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def read_csv_and_plot(filename):
    # Read CSV with tab delimiter since the data uses tabs
    df = pd.read_csv(filename, skiprows=1, delimiter='\t', names=['Force', 'PWM'])
    
    # Convert force column to numeric, forcing errors to NaN
    df['Force'] = pd.to_numeric(df['Force'], errors='coerce')
    
    # Convert force from grams to kilograms
    df['Force'] = df['Force'] / 1000
    
    # Filter positive forces for polynomial fitting
    positive_data = df[df['Force'] > 0.1]
    negative_data = df[df['Force'] <= -0.1]
    
    # Get the previous row and append it using concat
    previous_row_idx = df[df['Force'] > 0.1].index[0] - 1
    next_row_idx = df[df['Force'] <= -0.1].index[-1] + 1

    previous_row = pd.DataFrame([df.iloc[previous_row_idx]])
    next_row = pd.DataFrame([df.iloc[next_row_idx]])

    positive_data = pd.concat([previous_row, positive_data], ignore_index=True)
    negative_data = pd.concat([negative_data, next_row], ignore_index=True)
    
    print(negative_data)
    
    # Fit third order polynomial
    coefficients = np.polyfit(positive_data['Force'], positive_data['PWM'], 3)
    poly = np.poly1d(coefficients)

    coefficients_negative = np.polyfit(negative_data['Force'], negative_data['PWM'], 3)
    poly_negative = np.poly1d(coefficients_negative)
    
    # Generate points for smooth curve
    x_fit = np.linspace(positive_data['Force'].min(), positive_data['Force'].max(), 100)
    y_fit = poly(x_fit)

    x_fit_negative = np.linspace(negative_data['Force'].min(), negative_data['Force'].max(), 100)
    y_fit_negative = poly_negative(x_fit_negative)
    
    # Print coefficients in ascending order (0th to 3rd)
    coeff_ascending = coefficients[::-1]  # Reverse the array
    print("\nPolynomial coefficients (ascending order):")
    for i, c in enumerate(coeff_ascending):
        print(f"Order {i}: {c:.5f}")

    coeff_ascending_negative = coefficients_negative[::-1]  # Reverse the array
    print("\nPolynomial coefficients (ascending order) for negative data:")
    for i, c in enumerate(coeff_ascending_negative):
        print(f"Order {i}: {c:.5f}")
    
    # Create polynomial equation string
    eq = f"PWM = {coeff_ascending[0]:.2f}"
    for i, c in enumerate(coeff_ascending[1:], 1):
        sign = '+' if c >= 0 else '-'
        eq += f" {sign} {abs(c):.2f}x^{i}"
    print("\nPolynomial equation:")
    print(eq)

    eq_negative = f"PWM = {coeff_ascending_negative[0]:.2f}"
    for i, c in enumerate(coeff_ascending_negative[1:], 1):
        sign = '+' if c >= 0 else '-'
        eq_negative += f" {sign} {abs(c):.2f}x^{i}"
    print("\nPolynomial equation for negative data:")
    print(eq_negative)
    
    # Plotting
    plt.figure(figsize=(12, 8))
    
    # Plot original data
    plt.scatter(df['Force'], df['PWM'], color='blue', s=20, alpha=0.5, label='All Data Points')
    
    # Highlight positive force data used for fitting
    plt.scatter(positive_data['Force'], positive_data['PWM'], color='green', s=20, 
                alpha=0.7, label='Points Used for Fitting (Force > 0)')

    # Highlight negative force data used for fitting
    plt.scatter(negative_data['Force'], negative_data['PWM'], color='red', s=20, 
                alpha=0.7, label='Points Used for Fitting (Force <= 0)')
    
    # Plot polynomial fit
    plt.plot(x_fit, y_fit, 'r-', linewidth=2, 
             label=f'3rd Order Polynomial Fit\n{eq}')

    plt.plot(x_fit_negative, y_fit_negative, 'r-', linewidth=2,
             label=f'3rd Order Polynomial Fit\n{eq_negative}')
    
    plt.xlabel("Force (kg)")
    plt.ylabel("PWM (microseconds)")
    plt.title("PWM vs Force with Third Order Polynomial Fit")
    plt.grid(True, alpha=0.3)
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    plt.show()

# Example usage
filename = "motion/thruster_interface_asv/resources/ThrustMe_P1000_force_mapping.csv"
read_csv_and_plot(filename)