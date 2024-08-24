#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <ctime>
#include <pthread.h>
#include <semaphore.h>
#include <cmath>
#include <cstring>
#include <sys/wait.h>
#include <sys/mman.h>
#include <fcntl.h>

using namespace std;

// Structure defining the position of a robot
struct Position {
    int x;
    int y;
};

// Structure defining the estimated width of a robot's exit
struct RobotEstimate {
    int robotID;
    int estimatedWidth;
    bool accurateEstimation;
};

// Structure defining shared data among robots
struct SharedData {
    RobotEstimate estimates[50];
    int exitSize;
    int Total_width;
    sem_t mutex;
};

// Structure defining a queue for managing messages between robots
struct Queue {
    static const int MAX_SIZE = 50;
    RobotEstimate messages[MAX_SIZE];
    int front, rear;

    Queue() {
        front = -1;
        rear = -1;
    }

    bool isEmpty() {
        return front == -1 && rear == -1;
    }

    bool isFull() {
        return (rear + 1) % MAX_SIZE == front;
    }

    void enqueue(RobotEstimate msg) {
        if (isEmpty()) {
            front = rear = 0;
        } else {
            rear = (rear + 1) % MAX_SIZE;
        }
        messages[rear] = msg;
    }

    RobotEstimate dequeue() {
        RobotEstimate temp = messages[front];
        if (front == rear) {
            front = rear = -1;
        } else {
            front = (front + 1) % MAX_SIZE;
        }
        return temp;
    }
};

// Function representing the task performed by each robot
void* robotTask(void* arg);

int main() {
    const int numRobots = 50;
    const int shmSize = sizeof(SharedData);

    // Generating a unique key for shared memory
    const char* shmName = "/robot_shared_memory";
    int shm_fd = shm_open(shmName, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("Shared memory creation failed");
        return EXIT_FAILURE;
    }

    // Configure the size of the shared memory segment
    if (ftruncate(shm_fd, shmSize) == -1) {
        perror("Shared memory resize failed");
        return EXIT_FAILURE;
    }

    // Map the shared memory segment to the process's address space
    SharedData* shared_data = static_cast<SharedData*>(mmap(NULL, shmSize, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
    if (shared_data == MAP_FAILED) {
        perror("Mapping shared memory failed");
        return EXIT_FAILURE;
    }

    // Initializing shared data and synchronization objects
    shared_data->exitSize = 0;
    shared_data->Total_width = 0;
    sem_init(&(shared_data->mutex), 1, 1);

    // Seed for random number generation
    srand(static_cast<unsigned int>(time(nullptr)));

    pthread_t threads[numRobots];

    // Scheduling - Round-robin algorithm
    const int timeQuantum = 1; // Time quantum for each robot task in seconds

    int currentRobot = 0;
    while (currentRobot < numRobots) {
        pthread_create(&threads[currentRobot], nullptr, robotTask, shared_data);

        sleep(timeQuantum); // Allow each robot to execute for a time quantum
        currentRobot++;
    }

    // Joining threads after completion
    for (int i = 0; i < numRobots; ++i) {
        pthread_join(threads[i], nullptr);
    }

    sem_wait(&(shared_data->mutex));
    double average_width = (shared_data->exitSize > 0) ? static_cast<double>(shared_data->Total_width) / shared_data->exitSize : 0.0;
    cout << "Total Width (Average): " << average_width << endl;
    sem_post(&(shared_data->mutex));

    // Clean up: Unmap the shared memory and close the shared memory file descriptor
    munmap(shared_data, shmSize);
    close(shm_fd);
    shm_unlink(shmName);

    sem_destroy(&(shared_data->mutex));

    return 0;
}

void* robotTask(void* arg) {
    SharedData* shared_data = reinterpret_cast<SharedData*>(arg);

    int my_estimate;
    static int robotCounter = 0;
    int robotID = robotCounter++;
    Position robot_position;

    // Generating random positions for robots within the environment
    robot_position.x = rand() % 100;
    robot_position.y = rand() % 100;

    // Calculating the actual distance to the exit
    double actual_distance_to_exit = sqrt(pow(50 - robot_position.x, 2) + pow(50 - robot_position.y, 2));

    // Estimating exit size based on distance
    if (actual_distance_to_exit <= 5) {
        my_estimate = 50;
    } else if (actual_distance_to_exit <= 10) {
        my_estimate = rand() % 16 + 35;
    } else {
        my_estimate = rand() % 21 + 20;
    }

    // Checking the accuracy of the estimation
    bool accurate_estimation = false;
    if ((actual_distance_to_exit <= 5 && my_estimate >= 40 && my_estimate <= 60) ||
        (actual_distance_to_exit > 5 && actual_distance_to_exit <= 10 && my_estimate >= 35 && my_estimate <= 50) ||
        (actual_distance_to_exit > 10 && my_estimate >= 20 && my_estimate <= 40)) {
        accurate_estimation = true;
    }

    // Synchronizing access to shared data using semaphores
    sem_wait(&(shared_data->mutex));

    // Storing the robot's estimation data into shared memory
    RobotEstimate my_estimate_data;
    my_estimate_data.robotID = robotID;
    my_estimate_data.estimatedWidth = my_estimate;
    my_estimate_data.accurateEstimation = accurate_estimation;

    shared_data->estimates[shared_data->exitSize] = my_estimate_data;
    shared_data->exitSize++;
    shared_data->Total_width += my_estimate;

    sem_post(&(shared_data->mutex));

    // Creating a pipe for sending estimated exit size using IPC
    int pipefd[2];
    if (pipe(pipefd) == -1) {
        perror("Pipe creation failed");
        return nullptr;
    }

    // Writing estimated exit size to the pipe
    write(pipefd[1], &my_estimate_data, sizeof(my_estimate_data));
    close(pipefd[1]); // Close the write end of the pipe

    // Mutex for printing robot details
    static pthread_mutex_t console_mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_lock(&console_mutex);

    // Outputting details about the robot, actual and estimated widths
    cout << "Robot ID: " << robotID;
    cout << " at (" << robot_position.x << ", " << robot_position.y << ")";
    cout << " - Actual Width: " << actual_distance_to_exit;
    cout << ", Estimated Width: " << my_estimate;
    cout << ", Accuracy: " << (accurate_estimation ? "Correct" : "Incorrect") << endl;

    pthread_mutex_unlock(&console_mutex);

    return nullptr;
}

