num1 = input("Enter the first integer: ");
num2 = input("Enter the second integer: ");

result = processNumbers(num1, num2);

fprintf("The result is: %g\n", result);

function result = processNumbers(a,b)
    if mod(a,2) ~= 0 && mod(b,2) ~= 0
        result = a + b;
    elseif mod(a,2) == 0 && mod(b,2) == 0
        result = max(a,b) - min(a,b);
    else
        result = a * b;
    end
end