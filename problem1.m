firstNumber = input("Enter the first number: ");
secondNumber = input("Enter the second number: ");

result = firstNumber + secondNumber;

fprintf("%g + %g = %g\n", firstNumber, secondNumber, result);

A = [2 1; 1 -1];
b = [10; 2];

solution = A\b;

fprintf("Solution of simultaneous equations: x = %g, y = %g\n", solution(1), solution(2));