# Project Development Guidelines

1.  **Language Standard:** All code must adhere to the **C++17** standard. Leverage C++17 features where appropriate (e.g., `std::optional`, `std::string_view` if applicable, structured bindings).
2.  **Coding Style:**
    *   **Naming:** Use `snake_case` for variables and functions (e.g., `num_rows`, `calculate_value`). Use `PascalCase` for classes and structs (e.g., `Matrix`, `LayerConfig`). Use `UPPER_SNAKE_CASE` for constants and enum values (e.g., `MAX_ITERATIONS`). Member variables should have a consistent suffix (e.g., `rows_`, `data_`).
    *   **Formatting:** Aim for consistent indentation (e.g., 4 spaces) and line length (e.g., 100-120 chars). Consider using a formatter like ClangFormat later.
    *   **`const` Correctness:** Apply `const` liberally to variables, function parameters, and member functions that do not modify state.
    *   **Includes:** Include necessary headers directly. Avoid broad includes (like `<bits/stdc++.h>`). Order includes: Project headers (`"matrix.hpp"`), then standard library headers (`<vector>`, `<iostream>`).
3.  **Design Principles:**
    *   **Clarity over Premature Optimization:** Focus on writing clear, understandable, and correct code first. Optimization can be addressed later if profiling identifies bottlenecks.
    *   **RAII (Resource Acquisition Is Initialization):** Use RAII principles for managing resources (memory, files, etc.). Prefer smart pointers (`std::unique_ptr`, `std::shared_ptr`) over raw pointers where ownership is involved.
    *   **STL Usage:** Utilize the C++ Standard Library (STL) components (`std::vector`, `std::string`, algorithms, etc.) effectively.
    *   **Modularity:** Design classes and functions with clear responsibilities.
4.  **Dependencies:**
    *   **Standard Library First:** Prioritize using the C++ Standard Library.
    *   **Minimal External Libraries:** If external libraries are needed later, choose well-maintained, minimal options and justify their inclusion. Avoid large frameworks (as per the project goal).
5.  **Error Handling:**
    *   Use exceptions (`std::exception` hierarchy, e.g., `std::runtime_error`, `std::invalid_argument`, `std::out_of_range`) for reporting runtime errors that cannot be handled locally (like invalid input or out-of-bounds access).
    *   Use `assert` for checking internal invariants and preconditions during development (these are typically compiled out in release builds).
6.  **Testing:** (To be introduced)
    *   Unit tests should be written for core components and algorithms.
    *   Aim for good test coverage, including edge cases and error conditions.
7.  **Documentation:**
    *   Write clear comments for complex logic, design decisions, or non-obvious code sections.
    *   Use documentation comments (e.g., Doxygen style) for public APIs (classes, functions).
8.  **Version Control (Git):**
    *   Write clear, concise commit messages summarizing the changes.
    *   Commit frequently with logical units of work. 