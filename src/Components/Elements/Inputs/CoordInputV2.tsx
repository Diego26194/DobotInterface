import React, { useEffect, useState } from "react";
import { styled } from "@mui/material/styles";
import CompactsInput from "./CompactsInput";

const NoSpinnerInput = styled(CompactsInput)(({ theme }) => ({
  "& input::-webkit-outer-spin-button, & input::-webkit-inner-spin-button": {
    WebkitAppearance: "none",
    margin: 0,
  },
  "& input[type=number]": {
    MozAppearance: "textfield",
  },
}));

type CoordInputV2Props = {
  label: string;
  valMin: number;
  valMax: number;
  value: number;
  onChange: (val: number) => void;
  disabled?: boolean;
  onBlurCustom?: (newVal: number) => void;
};

const CoordInputV2: React.FC<CoordInputV2Props> = ({
  label,
  valMin,
  valMax,
  value,
  onChange,
  disabled = false,
  onBlurCustom,
}) => {
  const [displayVal, setDisplayVal] = useState(value.toFixed(2));

  // 🔄 Sincroniza el valor visible si el valor real cambia externamente
  useEffect(() => {
    setDisplayVal(value.toFixed(2));
  }, [value]);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    let val = e.target.value;

    // Permitir negativos y decimales
    val = val.replace(/[^0-9.-]/g, "");
    if (val.includes("-")) val = val.replace(/(?!^)-/g, "");
    const parts = val.split(".");
    if (parts.length > 2) val = parts[0] + "." + parts.slice(1).join("");

    setDisplayVal(val); // 🔹 Solo cambia lo que se ve, no el valor real todavía
  };

  const handleFocus = (e: React.FocusEvent<HTMLInputElement>) => {
    e.target.select(); // Selecciona todo el texto al enfocar
  };

  const handleBlur = () => {
    let num = parseFloat(displayVal);
    if (isNaN(num)) num = 0; // si queda vacío, se fuerza a 0

    // 🔹 Actualiza el valor real solo cuando sale del input
    onChange(num);
    setDisplayVal(num.toFixed(2));

    // Paso el nuevo valor al callback de blur
    if (onBlurCustom) onBlurCustom(num);
  };

  const numericVal = parseFloat(value.toString());
  const isOutOfRange =
    !isNaN(numericVal) && (numericVal < valMin || numericVal > valMax);

  return (
    <NoSpinnerInput
      type="text"
      label={label}
      value={displayVal}
      onChange={handleChange}
      onBlur={handleBlur}
      onFocus={handleFocus}
      disabled={disabled}
      inputProps={{
        inputMode: "decimal",
        pattern: "-?[0-9]*[.]?[0-9]*",
      }}
      error={isOutOfRange}
    />
  );
};

export default CoordInputV2;
