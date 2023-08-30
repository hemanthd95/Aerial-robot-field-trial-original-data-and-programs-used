#------------loading the library------------------

rm(list=ls(all=TRUE))
library(tidyverse)
library(ggplot2)
library(plyr)
library(dplyr)
library(tidyr)
library(spatialEco)# for finding the peaks and valleys
library(stringi)
#install.packages("stringi")

# Function to check if a string is valid UTF-8
is_valid_utf8 <- function(x) {
  tryCatch({
    stri_enc_isutf8(x)
  }, error = function(e) {
    return(FALSE)
  })
}

# Function to clean a text file from invalid multibyte strings
clean_text_file <- function(file_path) {
  text <- readLines(file_path, warn = FALSE, encoding = "UTF-8")
  valid_indices <- which(is_valid_utf8(text))
  clean_text <- text[valid_indices]
  writeLines(clean_text, con = file_path, useBytes = TRUE)
}

# Directory where your text files are located
directory_path <- "C:/Users/16822/OneDrive - USU/Documents/aerial_robot_8_2_2023/soil_moisture_data"


# Get a list of files in the directory
file_list <- list.files(directory_path, pattern = ".txt", full.names = TRUE)

# Loop through each file and clean it
for (file_path in file_list) {
  clean_text_file(file_path)
}
# Specify the folder path
folder_path <- "C:/Users/16822/OneDrive - USU/Documents/aerial_robot_8_2_2023/soil_moisture_data"

# Get the list of file names in the folder
file_names <- list.files(folder_path, pattern = "\\.txt$", full.names = TRUE)

numeric_part <- as.numeric(gsub("\\D", "", file_names))

# Sort file names based on the numeric part
sorted_file_names <- file_names[order(numeric_part)]

# Create an empty list to store the contents of each file
file_contents <- list()

# Loop over each file and read its contents
for (file_name in sorted_file_names) {
  file_content <- readLines(file_name)
  file_contents[[file_name]] <- file_content
}

# Print the contents of each file
for (i in seq_along(file_contents)) {
  file_name <- sorted_file_names[i]
  file_content <- file_contents[[file_name]]
  cat("File:", file_name, "\n")
  cat("Content:\n")
  cat(file_content, sep = "\n")
  cat("\n\n")
}
smc_string <- list()
smc_data <- data.frame()
for (i in 1:length(file_contents)) {
smc_string <- file_contents[[i]][seq(2,8,by=3)]
special_character <- "+"

# Split the selected lines into columns
columns <- lapply(smc_string, function(line) strsplit(line, split = paste0("\\", special_character), fixed = TRUE)[[1]])
data <- as.data.frame(do.call(rbind, columns))
smc_data <- rbind(smc_data,data)
}

smc_data <- separate(smc_data, V1, into = c("Sensor_address", "smc", "temperature","App_perm","bulk_conductivity","pore_conductivity"), sep = "\\+")
smc_data$location_number <- c(rep(1, 9), rep(2, 9),rep(3,9),rep(4,9),rep(5,9),
                              rep(6, 9), rep(7, 9),rep(8,9),rep(9,9),rep(10,9),
                              rep(11, 9), rep(12, 9))
                              
smc_data$replicate_number <- c(rep(1, 3), rep(2, 3),rep(3,3))
tt <- list()
travel_time <- data.frame()
for (i in 1:length(file_contents)) {
  tt <- file_contents[[i]][11]
  special_character <- "+"
  
  # Split the selected lines into columns
  columns <- lapply(tt, function(line) strsplit(line, split = paste0("\\", special_character), fixed = TRUE)[[1]])
  data <- as.data.frame(do.call(rbind, columns))
  travel_time <- rbind(travel_time,data)
  
}
travel_time <- separate(travel_time, V1, into = c("Sensor_address", "Epoxy_signal_start_time","max_second_derv_time", "probe_end_time_t3","travel_time","LDU"), sep = "\\+")
travel_time$location_number <- c(rep(1, 3), rep(2, 3),rep(3,3),rep(4,3),rep(5,3),
                                 rep(6, 3), rep(7, 3),rep(8,3),rep(9,3),rep(10,3),
                                 rep(11, 3), rep(12, 3))

# Specify the word you want to check
word_to_find <- "GPS"
#matching_lines <- grep(word_to_find, file_content, ignore.case = TRUE, value = TRUE)

gps_info <- list()
gps_data <- data.frame()
matching_lines <- list()
for (i in 1:length(file_contents)) {
  
# Use grep to find the line containing the word
matching_lines <- grep(word_to_find, file_contents[[i]], ignore.case = TRUE, value = TRUE)

# Check if the word exists in the file
if (length(matching_lines) > 0) {
  # Select the first occurrence of the word
  selected_line <- matching_lines[1]
  gps_info <- list(selected_line)
  data <- as.data.frame(do.call(rbind, gps_info))
  gps_data <- rbind(gps_data,data)
  
  #print(selected_line)
} 
else {
  print("Word not found in the file.")
}
}

word_to_find <- "Longitude"
#matching_lines <- grep(word_to_find, file_content, ignore.case = TRUE, value = TRUE)

gps_info_long <- list()
gps_data_long <- data.frame()
matching_lines <- list()
for (i in 1:length(file_contents)) {
  
  # Use grep to find the line containing the word
  matching_lines <- grep(word_to_find, file_contents[[i]], ignore.case = TRUE, value = TRUE)
  
  # Check if the word exists in the file
  if (length(matching_lines) > 0) {
    # Select the first occurrence of the word
    selected_line <- matching_lines[1]
    gps_info_long <- list(selected_line)
    data <- as.data.frame(do.call(rbind, gps_info_long))
    gps_data_long <- rbind(gps_data_long,data)
    
    #print(selected_line)
  } 
  else {
    print("Word not found in the file.")
  }
}

gps_data <- separate(gps_data, V1, into = c("GPS_info", "Latitude"), sep = ":")
gps_data_long <- separate(gps_data_long, V1, into = c("GPS_info", "Longitude"), sep = ":")
travel_time$latitude <- c(rep(gps_data[1,2], 3), rep(gps_data[2,2], 3),rep(gps_data[3,2],3),rep(gps_data[4,2],3),rep(gps_data[5,2],3),rep(gps_data[6,2], 3), rep(gps_data[7,2], 3),rep(gps_data[8,2],3),rep(gps_data[9,2],3),rep(gps_data[10,2],3),rep(gps_data[11,2],3),rep(gps_data[12,2],3))
travel_time$longitude <- c(rep(gps_data_long[1,2], 3), rep(gps_data_long[2,2], 3),rep(gps_data_long[3,2],3),rep(gps_data_long[4,2],3),rep(gps_data_long[5,2],3),rep(gps_data_long[6,2], 3), rep(gps_data_long[7,2], 3),rep(gps_data_long[8,2],3),rep(gps_data_long[9,2],3),rep(gps_data_long[10,2],3),rep(gps_data_long[11,2],3),rep(gps_data_long[12,2],3))
smc_data$latitude <- c(rep(gps_data[1,2], 9), rep(gps_data[2,2], 9),rep(gps_data[3,2],9),rep(gps_data[4,2],9),rep(gps_data[5,2],9),rep(gps_data[6,2], 9), rep(gps_data[7,2], 9),rep(gps_data[8,2],9),rep(gps_data[9,2],9),rep(gps_data[10,2],9),rep(gps_data[11,2],9),rep(gps_data[12,2],9))
smc_data$longitude <- c(rep(gps_data_long[1,2], 9), rep(gps_data_long[2,2], 9),rep(gps_data_long[3,2],9),rep(gps_data_long[4,2],9),rep(gps_data_long[5,2],9),rep(gps_data_long[6,2], 9), rep(gps_data_long[7,2], 9),rep(gps_data_long[8,2],9),rep(gps_data_long[9,2],9),rep(gps_data_long[10,2],9),rep(gps_data_long[11,2],9),rep(gps_data_long[12,2],9))



# Initialize an empty list to store the data frames
df_list <- list()

# Loop over each file
for (file_name in sorted_file_names) {
  # Read the text file
  file_content <- readLines(file_name)
  
  # Determine the indices of the start and end lines
  start_index <- 13
  end_index <- match("0", file_content) - 1
  
  # Extract the data lines between start and end indices
  data_lines <- file_content[start_index:end_index]
  
  # Create a data frame with a single column for the extracted data lines
  df <- data.frame(Content = data_lines)
  
  # Add the data frame to the list
  df_list[[file_name]] <- df
}

# New column name
new_column_name <- "Sensor_output"

# Function to rename column of a data frame
rename_column <- function(df, new_name) {
  colnames(df)[1] <- new_name
  return(df)
}

#df_list <- stringr::str_conv(df_list, "UTF-8")
#changing the column name in list of dataframes
df_list <- lapply(df_list, rename_column, new_name = new_column_name)

# Function to add a new column containing last three values from existing column
add_last_three_column <- function(df) {
  last_three_values <- substring(df$Sensor_output, nchar(df$Sensor_output) - 2)
  df$hex_dec_output <- rep(last_three_values,length.out =nrow(df))
  return(df)
}

# Apply the function to each data frame in the list
df_list <- lapply(df_list, add_last_three_column)

convert_hex_to_decimal <- function(df) {
  df$signal_amplitude <- strtoi(df$hex_dec_output,base = 16)
  return(df)
}

# Apply the function to each data frame in the list
df_list <- lapply(df_list, convert_hex_to_decimal)

start_year <- 1995
increment <- 15
num_rows <- max(sapply(df_list, nrow))
time_column <- seq(start_year, length.out = num_rows, by = increment)

# Create the dataframe
result_df <- data.frame(Time = time_column)

# Merge the third column from each data frame
for (i in seq_along(df_list)) {
  df <- df_list[[i]]
  if (ncol(df) >= 3) {
    df_merge <- data.frame(Time = time_column[1:nrow(df)], ThirdColumn = df[[3]])
  } else {
    df_merge <- data.frame(Time = time_column, ThirdColumn = rep(NA, num_rows))
  }
  result_df <- merge(result_df, df_merge, by = "Time", all = TRUE)
}
#removing rows containing NA
result_df <- result_df[complete.cases(result_df), ]

# Change column names from second column onwards
new_col_names <- paste0("Y_", "Amplitude_", 0:(ncol(result_df) - 2))
names(result_df)[2:ncol(result_df)] <- new_col_names

# Change the first column name
names(result_df)[1] <- "X_Time_0"

#creating a csv file for further travel time analysis
write_csv(result_df,"aerial_robot_8_2_2023_calib_verif.csv")
write_csv(smc_data,"aerial_robot_8_2_2023_smc_data_analysis.csv")
write_csv(travel_time,"aerial_robot_8_2_2023_travel_time_data.csv")