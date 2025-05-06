#include <iostream>
#include "matrix.hpp"
#include "perceptron_layer.hpp"
#include "activation.hpp"
#include "neural_network.hpp"
#include "loss.hpp"
#include "optimizer.hpp"
#include <stdexcept> 
#include <memory>   
#include <vector>   
#include <iomanip>  
#include <cstdlib>  
#include <ctime>    
#include "game.hpp"    
#include <cmath> 
#include <numeric> 
#include <random>  
#include <algorithm> 
#include <limits> 
#include <cstdio> 

// Forward declaration for the training function
void trainInterceptorModel(const std::string& model_filename, std::mt19937& rng, int max_epochs = 3000); 

// --- Function to Train the Interceptor AI Model ---
void trainInterceptorModel(const std::string& model_filename, std::mt19937& rng, int max_epochs) {
    std::cout << "Training Interceptor AI model (max " << max_epochs << " epochs)..." << std::endl;

    // --- Simulation/Game Constants (mirroring Game class for data generation) ---
    const double time_step = 0.1; // s
    const double max_accel = 5.0; // units/s^2
    const double world_w = 100.0;
    const double world_h = 100.0;
    const double missile_speed_train = 20.0; // units/s
    const double velocity_scale_train = 50.0; // Factor to scale velocity inputs

    // --- Network Configuration ---
    const int input_size = 4;  // rel_missile_x, rel_missile_y, missile_vx, missile_vy (scaled)
    const int hidden_size = 64; // Single hidden layer size
    const int output_size = 2; // accel_x, accel_y
    const double learning_rate = 0.01;

    // Create the optimizer
    auto optimizer = std::make_unique<SGD>(learning_rate);
    NeuralNetwork nn(std::move(optimizer), rng);

    // Add layers: Input -> ReLU(Hidden) -> Linear(Output)
    nn.addLayer(input_size, hidden_size, std::make_unique<ReLU>());     
    nn.addLayer(hidden_size, output_size, std::make_unique<Linear>()); // Output desired acceleration

    // --- Training Data Generation --- 
    const int num_total_samples = 10000; // Increased sample count
    Matrix all_inputs(num_total_samples, input_size);
    Matrix all_targets(num_total_samples, output_size);

    // Distributions for random initial states
    std::uniform_real_distribution<double> pos_x_dist(0.0, world_w);
    std::uniform_real_distribution<double> pos_y_dist(0.0, world_h);
    // Keep interceptor velocity 0 for simplicity in ideal calculation for now
    // std::uniform_real_distribution<double> vel_comp_dist(-5.0, 5.0); // If randomizing interceptor velocity later

    for (int i = 0; i < num_total_samples; ++i) {
        // --- Generate Random State ---
        Vec2 interceptor_pos{pos_x_dist(rng), pos_y_dist(rng)};
        Vec2 interceptor_vel{0.0, 0.0}; // Start stationary for easier target calculation
        
        Vec2 missile_pos{0.0, 0.0};
        Vec2 missile_target{0.0, 0.0};
        double dx_init, dy_init; // Declare variables outside the loop
        // Ensure missile and target aren't too close initially
        do { 
            missile_pos.x = pos_x_dist(rng);
            missile_pos.y = pos_y_dist(rng);
            missile_target.x = pos_x_dist(rng);
            missile_target.y = pos_y_dist(rng);
            dx_init = missile_pos.x - interceptor_pos.x;
            dy_init = missile_pos.y - interceptor_pos.y;
        } while (std::sqrt(dx_init*dx_init + dy_init*dy_init) < 10.0); // Ensure min starting distance

        Vec2 missile_vel{0.0, 0.0};
        double dx_m = missile_target.x - missile_pos.x;
        double dy_m = missile_target.y - missile_pos.y;
        double mag_m = std::sqrt(dx_m * dx_m + dy_m * dy_m);
        if (mag_m > 1e-6) {
            missile_vel.x = (dx_m / mag_m) * missile_speed_train;
            missile_vel.y = (dy_m / mag_m) * missile_speed_train;
        } else {
            // Random velocity if target is same as start
            double angle = std::uniform_real_distribution<double>(0, 2 * 3.1415926535)(rng);
            missile_vel.x = std::cos(angle) * missile_speed_train;
            missile_vel.y = std::sin(angle) * missile_speed_train;
        }

        // --- Calculate NN Inputs ---
        double rel_missile_x = missile_pos.x - interceptor_pos.x;
        double rel_missile_y = missile_pos.y - interceptor_pos.y;
        // Scale/Normalize inputs
        all_inputs(i, 0) = rel_missile_x / world_w; // Scale relative position by world size
        all_inputs(i, 1) = rel_missile_y / world_h;
        all_inputs(i, 2) = missile_vel.x / velocity_scale_train; // Scale velocity by arbitrary factor
        all_inputs(i, 3) = missile_vel.y / velocity_scale_train;

        // --- Calculate Ideal Target Acceleration (Direct Targeting) ---
        double delta_t = time_step; // Prediction timestep
        Vec2 missile_pos_future{0.0, 0.0};
        missile_pos_future.x = missile_pos.x + missile_vel.x * delta_t;
        missile_pos_future.y = missile_pos.y + missile_vel.y * delta_t;
        
        Vec2 required_disp{0.0, 0.0};
        required_disp.x = missile_pos_future.x - interceptor_pos.x;
        required_disp.y = missile_pos_future.y - interceptor_pos.y;
        
        // From Delta_P = V0*t + 0.5*A*t^2  => A = 2 * (Delta_P - V0*t) / t^2
        Vec2 ideal_accel{0.0, 0.0};
        if (delta_t > 1e-6) { 
             ideal_accel.x = 2.0 * (required_disp.x - interceptor_vel.x * delta_t) / (delta_t * delta_t);
             ideal_accel.y = 2.0 * (required_disp.y - interceptor_vel.y * delta_t) / (delta_t * delta_t);
        } else {
             ideal_accel.x = 0.0;
             ideal_accel.y = 0.0; // Avoid division by zero
        }

        // Clamp acceleration to maximum allowed
        ideal_accel.x = std::max<double>(-max_accel, std::min<double>(ideal_accel.x, max_accel));
        ideal_accel.y = std::max<double>(-max_accel, std::min<double>(ideal_accel.y, max_accel));

        // --- Store Target Outputs ---
        all_targets(i, 0) = ideal_accel.x;
        all_targets(i, 1) = ideal_accel.y;
    }

    // --- Data Splitting (Train/Validation) --- 
    const double validation_split_ratio = 0.20; 
    const int num_validation_samples = static_cast<int>(num_total_samples * validation_split_ratio);
    const int num_train_samples = num_total_samples - num_validation_samples;
    
    std::vector<int> indices(num_total_samples);
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), rng);
    Matrix train_inputs(num_train_samples, input_size);
    Matrix train_targets(num_train_samples, output_size);
    Matrix val_inputs(num_validation_samples, input_size);
    Matrix val_targets(num_validation_samples, output_size);
    
    for (int i = 0; i < num_train_samples; ++i) {
        int original_idx = indices[i];
        for (int j = 0; j < input_size; ++j) train_inputs(i, j) = all_inputs(original_idx, j);
        for (int j = 0; j < output_size; ++j) train_targets(i, j) = all_targets(original_idx, j);
    }
    
    for (int i = 0; i < num_validation_samples; ++i) {
        int original_idx = indices[num_train_samples + i]; 
        for (int j = 0; j < input_size; ++j) val_inputs(i, j) = all_inputs(original_idx, j);
        for (int j = 0; j < output_size; ++j) val_targets(i, j) = all_targets(original_idx, j);
    }

    // --- Training Loop with Early Stopping --- 
    std::unique_ptr<LossFunction> loss_func = std::make_unique<MeanSquaredError>();
    const int batch_size = 64; // Increased batch size slightly
    const int patience = 50; 
    const std::string best_model_temp_file = model_filename + ".best_temp";
    double best_val_loss = std::numeric_limits<double>::max();
    int epochs_no_improve = 0;
    int best_epoch = -1;
    
    std::cout << "Starting training (" << num_train_samples << " train, " << num_validation_samples << " val samples)." << std::endl;
    
    for (int e = 0; e < max_epochs; ++e) {
        double total_epoch_loss = 0.0;
        int batches_processed = 0;
        std::vector<int> train_indices(num_train_samples);
        std::iota(train_indices.begin(), train_indices.end(), 0);
        std::shuffle(train_indices.begin(), train_indices.end(), rng);
        
        for (size_t i = 0; i < static_cast<size_t>(num_train_samples); i += batch_size) {
            size_t current_batch_size = std::min<size_t>(static_cast<size_t>(batch_size), static_cast<size_t>(num_train_samples) - i);
            if (current_batch_size == 0) continue;
            
            Matrix batch_inputs(static_cast<int>(current_batch_size), input_size);
            Matrix batch_targets(static_cast<int>(current_batch_size), output_size);
            
            for (size_t k = 0; k < current_batch_size; ++k) {
                int sample_idx = train_indices[i + k];
                for(int j=0; j<input_size; ++j) batch_inputs(static_cast<int>(k), j) = train_inputs(sample_idx, j);
                for(int j=0; j<output_size; ++j) batch_targets(static_cast<int>(k), j) = train_targets(sample_idx, j);
            }
            
            total_epoch_loss += nn.train(batch_inputs, batch_targets, *loss_func);
            batches_processed++;
        }
        
        double avg_train_loss = (batches_processed > 0) ? (total_epoch_loss / batches_processed) : 0.0;
        Matrix val_output = nn.forward(val_inputs);
        double current_val_loss = loss_func->calculate(val_output, val_targets);
        
        if ((e + 1) % 100 == 0) { 
             std::cout << "Epoch [" << (e + 1) << "/" << max_epochs 
                       << "], Train Loss: " << avg_train_loss 
                       << ", Val Loss: " << current_val_loss << std::endl;
        }
        
        if (current_val_loss < best_val_loss) {
            best_val_loss = current_val_loss;
            epochs_no_improve = 0;
            best_epoch = e + 1;
            nn.saveModel(best_model_temp_file);
        } else {
            epochs_no_improve++;
        }
        
        if (epochs_no_improve >= patience) {
            std::cout << "\nEarly stopping triggered after " << e + 1 << " epochs." << std::endl;
            std::cout << "Best validation loss " << best_val_loss << " achieved at epoch " << best_epoch << "." << std::endl;
            break; 
        }
    }
    
    if (best_epoch != -1 && best_epoch < (max_epochs)) {
        std::cout << "Loading best model from epoch " << best_epoch << " (Val Loss: " << best_val_loss << ")" << std::endl;
        try {
             auto final_optimizer = std::make_unique<SGD>(learning_rate); 
             NeuralNetwork best_nn(std::move(final_optimizer), rng);
             best_nn.loadModel(best_model_temp_file);
             best_nn.saveModel(model_filename);
             std::cout << "Best model loaded and saved to " << model_filename << std::endl;
        } catch (const std::exception& e) {
             std::cerr << "Error loading/saving best model: " << e.what() << std::endl;
             std::cerr << "Saving model from the last epoch instead." << std::endl;
             nn.saveModel(model_filename); 
        }
    } else {
         std::cout << "Training completed " << max_epochs << " epochs." << std::endl;
         if(best_epoch != -1) {
            std::cout << "Best validation loss " << best_val_loss << " achieved at epoch " << best_epoch << "." << std::endl;
         } else {
            std::cout << "No improvement in validation loss observed. Saving model from final epoch." << std::endl;
         }
         nn.saveModel(model_filename);
         std::cout << "Model saved to " << model_filename << std::endl;
    }
    
    std::remove(best_model_temp_file.c_str());
    std::cout << "Temporary best model file deleted." << std::endl;
    std::cout << "Training function finished." << std::endl;
}

// Removed old calculateIdealMove function

int main(int argc, char *argv[]) {
    std::random_device rd;
    std::mt19937 main_rng(rd());

    const std::string model_file = "interceptor_model.bin"; // New model filename

    bool train_mode = false;
    if (argc > 1) {
        std::string arg1 = argv[1];
        if (arg1 == "train") {
            train_mode = true;
        }
    }

    if (train_mode) {
        std::cout << "--- Running in Training Mode (Interceptor) ---" << std::endl;
        trainInterceptorModel(model_file, main_rng); // Call the new training function
        std::cout << "Training finished." << std::endl;
    } else {
        std::cout << "--- Running in Gameplay Mode (Interceptor) --- " << std::endl;
        std::cout << "Loading model: " << model_file << std::endl;
        try {
            Game game(model_file, main_rng); // Create Game instance with model and RNG
            game.run(); // Run the game loop
            std::cout << "Game session completed." << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error during game initialization or run: " << e.what() << std::endl;
            std::cerr << "Ensure the model file '" << model_file << "' exists. Run with 'train' argument to create it." << std::endl;
            return 1;
        }
    }

    return 0;
} 