#include "test_utils.h"
#include "cli/cli.h"

extern struct cli_handle cli;

/**
 * @brief print a matrix through the CLI
 *
 * Given a buffer and the dimensions of the matrix to be printed, print
 * the matrix to the CLI as a comma-separated string.
 * @param matrix pointer to matrix to print
 * @param rows number of rows in matrix
 * @param cols number of columns in matrix
 */
void print_matrix(float64_t *matrix, uint16_t rows, uint16_t cols) {
  for (uint16_t j = 0; j < rows; j++) {
    for (uint16_t i = 0; i < cols; i++) {
      cli_transmit(&cli, "%.10f,", matrix[j * cols + i]);
    }
    cli_transmit(&cli, "\r\n");
  }
}

/**
 * @brief determine if two buffers are the same within a tolerance
 *
 * Compare two buffers of float64_ts and determine if each element within those
 * buffers is the same within a specified tolerance calculated as: fabs(buf_1[i]
 * - buf_2[i]) < tol.
 * @param buf_1 pointer to first float64_t buffer
 * @param buf_2 pointer to second float64_t buffer
 * @param num_el number of elements in float64_t buffers
 * @param tol tolerance to check against
 * @return 1 if every element is within tolerance, 0 otherwise
 */
uint8_t compare(float64_t *buf_1, float64_t *buf_2, uint16_t num_el,
                float64_t tol) {
  for (uint16_t i = 0; i < num_el; i++) {
    float64_t err = fabs(buf_1[i] - buf_2[i]);
    if (err > tol) {
      return 0;
    }
  }

  return 1;
}